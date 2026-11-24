#include "rl_stair.h"
#include "rl_control.h"
#include "rl_policy_core.h"
#include "leg_const.hpp"

#include <math.h>
#include <string.h>

#define SAMPLE_MAX_GAP_MS 10u
#define ANCHOR_MAX_AGE_MS 250u
#define WHEEL_SHOCK_WINDOW_MS 20u
#define ROBOT_WEIGHT_N ((M_B + 2.0f * (M_L + M_W)) * G)

int RLStairBuildObservation(const RLMotion31Snapshot *s, float o[31])
{
    if (RLMotion31ValidateSnapshot(s) != 0) return -1;
    const float command[3] = {0.0f, 0.0f, RL_STAIR_GOAL_M};
    return RLPolicyEncode(s->gyro_model_rad_s, s->gravity_model, s->joint_position_rad,
                          s->joint_velocity_rad_s, s->previous_target, command, 1u, 0u, o);
}

float RLStairWheelLoad(float phi1, float phi4, float phi2, float phi3,
                       float tau1, float tau4, float pitch, float roll)
{
    /* Implicitly differentiate the two fixed lower-link lengths at the hub.
       The Jacobian maps joint rates to (rearward, downward) hub velocity. */
    const float det_link = sinf(phi3 - phi2);
    if (!isfinite(det_link) || fabsf(det_link) < 0.08f) return NAN;
    const float k1 = L_1 * sinf(phi2 - phi1) / det_link;
    const float k4 = L_4 * sinf(phi3 - phi4) / det_link;
    const float jx1 = sinf(phi3) * k1, jy1 = -cosf(phi3) * k1;
    const float jx4 = -sinf(phi2) * k4, jy4 = cosf(phi2) * k4;
    const float det = jx1 * jy4 - jx4 * jy1;
    if (!isfinite(det) || fabsf(det) < 0.001f) return NAN;
    /* J^T f = tau; projection of downwards push estimates upwards reaction. */
    const float fx = (jy4 * tau1 - jy1 * tau4) / det;
    const float fy = (jx1 * tau4 - jx4 * tau1) / det;
    return (-sinf(pitch) * fx + cosf(pitch) * fy) * cosf(roll);
}

static int known_pose(const RLStairSample *s)
{
    return isfinite(s->pitch) && isfinite(s->roll) && isfinite(s->pitch_rate) &&
           isfinite(s->omega_squared) && isfinite(s->accel_up) &&
           isfinite(s->swing[0]) && isfinite(s->swing[1]) &&
           isfinite(s->wheel_to_root_m[0]) && isfinite(s->wheel_to_root_m[1]);
}

static int load_known(const RLStairSample *s)
{
    return isfinite(s->wheel_up_estimate_n[0]) && isfinite(s->wheel_up_estimate_n[1]);
}

static int loaded(const RLStairSample *s)
{
    return load_known(s) &&
           s->wheel_up_estimate_n[0] >= 2.0f && s->wheel_up_estimate_n[1] >= 2.0f &&
           s->wheel_up_estimate_n[0] + s->wheel_up_estimate_n[1] >= 0.60f * ROBOT_WEIGHT_N;
}

static int level(const RLStairSample *s)
{
    return fabsf(s->pitch) <= 5.0f * DEG_2_RAD && fabsf(s->roll) <= 5.0f * DEG_2_RAD &&
           s->omega_squared <= 0.35f * 0.35f;
}

static int impact(const RLStairSample *s)
{
    const float rear = fmaxf(s->swing[0], s->swing[1]);
    return s->pitch > 6.0f * DEG_2_RAD && rear > 14.0f * DEG_2_RAD &&
           (s->pitch_rate > 10.0f * DEG_2_RAD ||
            (isfinite(s->vx_estimate) && s->vx_cmd - s->vx_estimate > 0.20f) ||
            s->pitch > 12.0f * DEG_2_RAD || rear > 20.0f * DEG_2_RAD);
}

static int quick_impact(RLStairHandoff *h, const RLStairSample *s, uint32_t *first)
{
    const uint32_t now = s->tick_ms;
    const uint32_t dt_ms = now-h->last_sample_tick;
    if (!h->have_sample || dt_ms == 0u || dt_ms > SAMPLE_MAX_GAP_MS) {
        h->wheel_shock_mask = 0u;
        return 0;
    }
    /* A wheel's circumferential speed collapses before large body tilt. Use
       it only on an established straight approach, with independent pitch
       rate and rearward movement of that same leg as corroboration. */
    const int aligned = fabsf(h->wheel_speed_last[0] - h->wheel_speed_last[1]) <= 0.45f;
    int found = 0;
    for (unsigned i=0; i<2; ++i) {
        const unsigned bit = 1u << i;
        if ((uint32_t)(now-h->shock_tick[i]) > WHEEL_SHOCK_WINDOW_MS)
            h->wheel_shock_mask &= (uint8_t)~bit;
        if (!(h->wheel_shock_mask & bit) && aligned && h->wheel_speed_last[i] >= 1.0f &&
            isfinite(s->wheel_forward_mps[i]) &&
            h->wheel_speed_last[i] - s->wheel_forward_mps[i] >= 0.60f) {
            h->wheel_shock_mask |= (uint8_t)bit;
            h->shock_tick[i] = now;
            h->swing_at_shock[i] = h->swing_last[i];
        }
        if ((h->wheel_shock_mask & bit) && s->pitch_rate >= 0.30f &&
            fabsf(s->roll) <= 0.12f && s->pitch >= -0.14f &&
            s->vx_estimate >= 0.60f && s->swing[i] >= 0.04f &&
            s->swing[i] - h->swing_at_shock[i] >= 0.018f) {
            if (!found || (uint32_t)(now-h->shock_tick[i]) > (uint32_t)(now-*first))
                *first = h->shock_tick[i];
            found = 1;
        }
    }
    return found;
}

static int on_tread(const RLStairHandoff *h)
{
    if (!h->have_anchor) return 0;
    for (unsigned i=0; i<2; ++i)
        if (!(h->tread_estimate_m[i] >= 0.12f && h->tread_estimate_m[i] <= 0.28f))
            return 0;
    return 1;
}

static int raised_for_motion(const RLStairSample *s)
{
    /* Motion can recover residual body motion once it has two loaded wheels
       and usable standing height. Keep its existing virtual-leg eligibility
       so a transfer will not immediately fall out of RL into LQR. */
    const float body_m = 0.5f*(s->wheel_to_root_m[0]+s->wheel_to_root_m[1]);
    return body_m >= RL_STAIR_RETURN_MIN_BODY_M &&
           fabsf(s->swing[0]) <= RL_CONTROL_MOTION_SWING_LIMIT_RAD &&
           fabsf(s->swing[1]) <= RL_CONTROL_MOTION_SWING_LIMIT_RAD;
}

static void clear_support(RLStairSupportWindow *w)
{
    /* Only the live spans are read. Avoid touching the whole ring on reset. */
    w->duration_ms = w->loaded_ms = w->gap_ms = 0u;
    w->head = w->count = w->started = w->last_loaded = 0u;
}

static void trim_support(RLStairSupportWindow *w, unsigned excess_ms)
{
    while (excess_ms) {
        RLStairSupportSpan *span = &w->spans[w->head];
        const unsigned drop = excess_ms < span->duration_ms ? excess_ms : span->duration_ms;
        w->duration_ms -= drop;
        if (span->loaded) w->loaded_ms -= drop;
        excess_ms -= drop;
        span->duration_ms -= drop;
        if (!span->duration_ms) {
            w->head = (w->head+1u) % RL_STAIR_SUPPORT_SPANS;
            --w->count;
        }
    }
}

static int support_confirmed(RLStairHandoff *h, const RLStairSample *s, unsigned dt_ms)
{
    RLStairSupportWindow *w = &h->support;
    if (!h->stair_adopted || !load_known(s)) {
        clear_support(w);
        return 0;
    }
    /* Count upper support as soon as both wheels take weight, including
       while the legs are still lifting the body to the handoff height. */
    const int load_now = on_tread(h) && loaded(s);
    if (!dt_ms || dt_ms > SAMPLE_MAX_GAP_MS) clear_support(w);
    if (!w->started) {
        w->started = w->last_loaded = (uint8_t)load_now;
        return 0;
    }

    w->gap_ms = load_now || w->last_loaded ? 0u : w->gap_ms+dt_ms;
    const unsigned duration = w->duration_ms+dt_ms;
    if (duration > RL_STAIR_SUPPORT_WINDOW_MS)
        trim_support(w,duration-RL_STAIR_SUPPORT_WINDOW_MS);
    if (w->count == RL_STAIR_SUPPORT_SPANS) {
        /* A schedule without capacity for the full history cannot certify it. */
        clear_support(w);
        w->started = w->last_loaded = (uint8_t)load_now;
        return 0;
    }
    /* Credit time from the sample that opened the interval. A recovered
       wheel must not retroactively earn the preceding unloaded interval. */
    const unsigned tail = (w->head+w->count) % RL_STAIR_SUPPORT_SPANS;
    w->spans[tail] = (RLStairSupportSpan){.duration_ms=(uint8_t)dt_ms,.loaded=w->last_loaded};
    ++w->count;
    w->duration_ms += dt_ms;
    if (w->last_loaded) w->loaded_ms += dt_ms;
    w->last_loaded = (uint8_t)load_now;
    return load_now && w->loaded_ms >= RL_STAIR_SUPPORT_MIN_MS && raised_for_motion(s);
}

static void sample_height(RLStairHandoff *h, const RLStairSample *s)
{
    const uint32_t dt_ms = s->tick_ms - h->last_sample_tick;
    const int contiguous = h->have_sample && dt_ms > 0u && dt_ms <= SAMPLE_MAX_GAP_MS;
    const float dt = (float)dt_ms * 0.001f;
    if (!contiguous) {
        h->condition = 0u;
        if (!RLStairEngaged(h)) h->have_anchor = 0u;
    }
    const float height = 0.5f * (s->wheel_to_root_m[0] + s->wheel_to_root_m[1]);
    const int bilateral = RLStairEngaged(h) && level(s) && loaded(s) &&
        fabsf(s->accel_up-h->bias) <= 1.0f && fabsf(height-RL_STAIR_GOAL_M) <= 0.04f &&
        fabsf(s->wheel_to_root_m[0]-s->wheel_to_root_m[1]) <= 0.04f;
    /* Establish the lower-floor datum during the level, loaded approach. Once
       the nose pitches down, keep it in the same short IMU integration frame. */
    if (!RLStairEngaged(h) && level(s) && loaded(s) && height > 0.30f &&
        fabsf(s->accel_up) <= 1.0f) {
        const float sample_vz = (contiguous && h->have_anchor &&
                                 h->anchor_tick == h->last_sample_tick)
            ? (height-h->root_m)/dt : 0.0f;
        h->root_vz = 0.85f*h->root_vz + 0.15f*fminf(0.35f, fmaxf(-0.35f, sample_vz));
        h->root_m = height;
        h->bias = fminf(0.25f, fmaxf(-0.25f, 0.98f*h->bias + 0.02f*s->accel_up));
        h->anchor_tick = s->tick_ms;
        h->have_anchor = 1u;
    } else if (contiguous && h->have_anchor && bilateral && h->last_bilateral) {
        /* When both loaded hubs constrain the same plane, track changes in
           relative height without integrating IMU bias for the entire dwell. */
        const float dh = height-h->height_last;
        h->root_m += dh;
        h->root_vz = fminf(0.35f, fmaxf(-0.35f, dh/dt));
    } else if (contiguous && h->have_anchor) {
        const float a = 0.5f * (s->accel_up + h->accel_last) - h->bias;
        h->root_m += dt * (h->root_vz + 0.5f * dt * a);
        h->root_vz += dt * a;
    }
    h->accel_last = s->accel_up;
    h->height_last = height;
    h->last_bilateral = (uint8_t)bilateral;
    memcpy(h->wheel_speed_last,s->wheel_forward_mps,sizeof(h->wheel_speed_last));
    memcpy(h->swing_last,s->swing,sizeof(h->swing_last));
    h->last_sample_tick = s->tick_ms;
    h->imu_sequence = s->imu_sequence;
    h->have_sample = 1u;
    for (unsigned i=0; i<2; ++i) h->tread_estimate_m[i] = h->root_m - s->wheel_to_root_m[i];
}

static int condition_for(RLStairHandoff *h, uint32_t now, int value, uint32_t hold)
{
    if (!value) {
        h->condition = 0u;
        return 0;
    }
    if (!h->condition) {
        h->condition_tick = now;
        h->condition = 1u;
    }
    return (uint32_t)(now - h->condition_tick) >= hold;
}

void RLStairStop(RLStairHandoff *h, RLStairFault fault)
{
    h->fault = fault;
    h->phase = RL_STAIR_STOPPED;
}

void RLStairReset(RLStairHandoff *h, int armed)
{
    memset(h, 0, sizeof(*h));
    if (armed) h->phase = RL_STAIR_WAIT_RELEASE;
}

void RLStairUpdate(RLStairHandoff *h, const RLStairSample *s)
{
    if (h->phase == RL_STAIR_STOPPED) return;
    if (h->phase == RL_STAIR_WAIT_RELEASE) {
        if (!s->armed) {
            h->phase = RL_STAIR_IDLE;
            h->condition = h->have_sample = h->have_anchor = h->stair_adopted = 0u;
            h->wheel_shock_mask = h->last_bilateral = 0u;
            h->root_vz = h->bias = 0.0f;
        }
        return;
    }

    const uint32_t now = s->tick_ms;
    if (RLStairEngaged(h)) {
        if ((uint32_t)(now - h->impact_tick) >= RL_STAIR_TIMEOUT_MS) {
            RLStairStop(h, RL_STAIR_TIMEOUT);
            return;
        }
        if (!s->link_fresh || !s->feedback_fresh || !known_pose(s) ||
            (uint32_t)(now - h->last_sample_tick) > SAMPLE_MAX_GAP_MS) {
            RLStairStop(h, RL_STAIR_STALE);
            return;
        }
        /* Wider than the transient poses in the Stair contract. */
        if (fabsf(s->pitch) > 0.90f || fabsf(s->roll) > 0.52f ||
            fabsf(s->swing[0]) > PI/2.0f || fabsf(s->swing[1]) > PI/2.0f) {
            RLStairStop(h, RL_STAIR_POSE);
            return;
        }
    } else if (!s->armed || !s->can_enter || !s->link_fresh || !s->feedback_fresh ||
               !known_pose(s)) {
        h->condition = h->have_sample = h->have_anchor = h->wheel_shock_mask = 0u;
        h->root_vz = h->bias = 0.0f;
        return;
    }

    if (h->have_sample && s->imu_sequence == h->imu_sequence) {
        h->condition = h->wheel_shock_mask = 0u;
        clear_support(&h->support);
        if (h->phase == RL_STAIR_RETURN) {
            h->phase = RL_STAIR_ACTIVE;
            h->supported_tick = 0;
        }
        return;
    }

    const uint32_t sample_dt_ms = now-h->last_sample_tick;
    uint32_t first_shock = now;
    const int quick = h->phase == RL_STAIR_IDLE && quick_impact(h, s, &first_shock);
    sample_height(h, s);
    if (h->phase == RL_STAIR_IDLE) {
        const int arrived = h->have_anchor && (uint32_t)(now-h->anchor_tick) <= ANCHOR_MAX_AGE_MS;
        if ((arrived && quick) || condition_for(h, now, arrived && impact(s), RL_STAIR_IMPACT_MS)) {
            h->impact_start_tick = quick ? first_shock : h->condition_tick;
            h->impact_tick = now;
            h->stair_adopted_tick = h->supported_tick = h->motion_adopted_tick = 0u;
            h->phase = RL_STAIR_ACTIVE;
            h->condition = 0u;
            clear_support(&h->support);
        }
    } else if (support_confirmed(h,s,sample_dt_ms)) {
        if (h->phase == RL_STAIR_ACTIVE) {
            h->supported_tick = now;
            h->phase = RL_STAIR_RETURN;
        }
    } else if (h->phase == RL_STAIR_RETURN) {
        /* Recheck support and receiver eligibility before applying Motion.
           Recent load history stays available if the pose recovers. */
        h->phase = RL_STAIR_ACTIVE;
        h->supported_tick = 0u;
    }
}

void RLStairOnAdopt(RLStairHandoff *h, uint8_t actor, uint32_t now)
{
    if (h->phase == RL_STAIR_ACTIVE && actor == RL_ACTOR_STAIR && !h->stair_adopted) {
        h->stair_adopted_tick = now;
        h->stair_adopted = 1u;
    } else if (h->phase == RL_STAIR_RETURN && actor == RL_ACTOR_MOTION) {
        h->motion_adopted_tick = now;
        h->phase = RL_STAIR_WAIT_RELEASE;
    }
}
