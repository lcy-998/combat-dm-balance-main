# 电控分支导航

核对日期：2026-11-14。分支名保留兼容现有拉取命令；本表是功能备注，不代表历史分支已经合并或废弃。Git分支description是本地元数据，不会随push传播；本文件和根README可随当前集成分支传播。

## 按任务选择

| 用途标签 | 完整分支名 | 内容与选择建议 |
|---|---|---|
| **当前自动起身试验** | **`feature/rl-supported015-recovery2100`** | **Recovery2100起身→31D256 Motion015自动接管；本体准备100ms；含两套C权重和HEX/BIN。当前试验用这个。** |
| Motion基线 | `feature/rl-motion31-supported015` | 单独015 Motion，31D256，站立/运动与256权重转换工具；不含本次2100自动接管集成。远端核对基点7f89e6d。 |
| 旧Recovery | `feature/rl-tjb-recover` | 历史起身实现；不是2100集成版。不要因名字有recover就当作本次交付。 |
| 历史Motion | `feature/rl-motion31-shadow` | 早期31D观测/网络接入；不是当前集成版。 |
| 历史输入接入 | `feature/rl-input-shadow` | 更早的观测/网络接入。当前仅远端引用。 |
| 独立辨识 | `feature/hip-sine-identification` | 髋关节辨识扫频；不是起身试验固件。当前仅远端引用。 |
| 传统控制基线 | `main` | 传统控制；不能把main理解为本次RL最新版。当前远端基点656fcfa。 |
| 本地Jump开发 | `feature/rl-jump-direct-v1` | direct方案工作区，未发布到远端；未提交内容须由对应任务核对。 |
| 本地Jump开发 | `feature/rl-jump-simple-v1` | simple方案工作区，未发布到远端；未提交内容须由对应任务核对。 |
| 本地Jump开发 | `feature/rl-jump-unified48-v1` | unified48方案工作区，未发布到远端；未提交内容须由对应任务核对。 |
| 只读审阅快照 | `review/rl-supported015-remote` | 曾用于审阅015，检出786a2c5；不是最新Motion或当前试验版。 |

## 目录不等于分支

以下为本机worktree快照。不要在已有脏工作区直接切分支、pull或覆盖文件。

| 本机目录（均在/home/kellen下） | 实际分支/状态 |
|---|---|
| **`combat-dm-balance-supported015-recovery2100`** | **当前集成分支；本次交付正式工作区** |
| `combat-dm-balance-motion31-supported015` | 本地015分支落后远端，且有未提交修改；不是最新远端源码 |
| `combat-dm-balance-main` | 实际为`feature/rl-motion31-shadow`，有未提交修改；目录名main容易误导 |
| `combat-dm-balance-tjb-recover` | 旧Recovery分支，有未提交草稿；不是本次已发布版本 |
| `combat-dm-balance-supported015-remote` | review快照工作区 |
| `combat-dm-balance-control-integration` | detached HEAD，4ba73bc，有未提交修改；不是可直接选用的命名分支 |
| `combat-dm-balance-jump-direct-v1` | 对应direct本地开发分支，有未提交修改 |
| `combat-dm-balance-jump-simple-v1` | 对应simple本地开发分支，有未提交修改 |
| `combat-dm-balance-jump-unified48-v1` | 对应unified48本地开发分支，有未提交修改 |

## 核对命令

```sh
git branch --show-current
git log -1 --oneline
git status --short
git branch -avv
git worktree list
# 本机用途备注；其他机器没有这些description时以本导航为准。
git config --get-regexp '^branch\..*\.description$'
```

本次只增加备注与导航，没有重命名、删除分支，没有移动worktree，没有覆盖其他工作区未提交内容。远端分支若以后要统一改名，需同步更新协作者、CI和操作说明；不建立两套同功能活动分支。
