#!/usr/bin/env python3
"""Replace a fixed X-CUBE-AI network's FP32 weights from a compatible ONNX.

Usage:
  python tools/update_xcube_ai_weights.py --target tjb_network path/to/actor.onnx
  python tools/update_xcube_ai_weights.py --target network path/to/policy.onnx

Only the learned FP32 initializers may change for the selected target.
"""

from __future__ import annotations

import argparse
import math
import re
import struct
from pathlib import Path

try:
    import onnx
    from onnx import numpy_helper
except ImportError as exc:
    raise SystemExit("Missing dependency: install it with `python -m pip install onnx`.") from exc


TARGETS = {
    "tjb_network": {
        "dims": [(256, 31), (128, 256), (64, 128), (6, 64)],
        "ops": ["Gemm", "Elu", "Gemm", "Elu", "Gemm", "Elu", "Gemm", "Tanh"],
        "input": 31,
        "weight_bytes": 198936,
    },
    "network": {
        "dims": [(128, 25), (128, 128), (6, 128)],
        "ops": ["Gemm", "Relu", "Gemm", "Relu", "Gemm"],
        "input": 25,
        "weight_bytes": 82456,
    },
}


def array_pattern(target):
    return re.compile(
        rf"(const\s+ai_u64\s+s_{re.escape(target)}_weights_array_u64\[\d+\]\s*=\s*\{{)(.*?)(\n\}};)",
        re.S,
    )


def dims(value_info):
    return [dim.dim_value for dim in value_info.type.tensor_type.shape.dim]


def initializer_bytes(tensor):
    if tensor.data_type != onnx.TensorProto.FLOAT:
        raise ValueError(f"initializer {tensor.name!r} is not float32")
    raw = tensor.raw_data
    if not raw:
        raw = numpy_helper.to_array(tensor).astype("<f4", copy=False).tobytes()
    return raw


def extract_weights(model, target="tjb_network"):
    spec = TARGETS[target]
    onnx.checker.check_model(model)
    graph = model.graph
    inputs = [x for x in graph.input if x.name not in {i.name for i in graph.initializer}]
    outputs = list(graph.output)
    if len(inputs) != 1 or dims(inputs[0]) != [1, spec["input"]]:
        raise ValueError(f"expected input shape [1, {spec['input']}], got {[dims(x) for x in inputs]}")
    if len(outputs) != 1 or dims(outputs[0]) != [1, 6]:
        raise ValueError(f"expected output shape [1, 6], got {[dims(x) for x in outputs]}")
    ops = [node.op_type for node in graph.node]
    if ops != spec["ops"]:
        raise ValueError(f"expected operators {spec['ops']}, got {ops}")
    if inputs[0].type.tensor_type.elem_type != onnx.TensorProto.FLOAT or outputs[0].type.tensor_type.elem_type != onnx.TensorProto.FLOAT:
        raise ValueError("input and output must be float32")

    tensors = {x.name: x for x in graph.initializer}
    if len(tensors) != 2 * len(spec["dims"]):
        raise ValueError(f"expected {2 * len(spec['dims'])} initializers, got {len(tensors)}")
    result = bytearray()
    expected_count = 0
    previous = inputs[0].name
    for index, expected_dim in enumerate(spec["dims"]):
        node = graph.node[index * 2]
        if node.input[0] != previous:
            raise ValueError(f"Gemm {node.name!r} is not connected to the preceding layer")
        attributes = {a.name: onnx.helper.get_attribute_value(a) for a in node.attribute}
        if attributes.get("alpha", 1.0) != 1.0 or attributes.get("beta", 1.0) != 1.0 or attributes.get("transA", 0) != 0 or attributes.get("transB", 0) != 1:
            raise ValueError(f"Gemm {node.name!r} has incompatible attributes: {attributes}")
        if len(node.input) < 3 or node.input[1] not in tensors or node.input[2] not in tensors:
            raise ValueError(f"Gemm {node.name!r} does not have constant weight and bias")
        weight, bias = tensors[node.input[1]], tensors[node.input[2]]
        if list(weight.dims) != list(expected_dim):
            raise ValueError(f"{weight.name} has shape {list(weight.dims)}, expected {expected_dim}")
        if list(bias.dims) != [expected_dim[0]]:
            raise ValueError(f"{bias.name} has shape {list(bias.dims)}, expected {[expected_dim[0]]}")
        result.extend(initializer_bytes(weight))
        result.extend(initializer_bytes(bias))
        expected_count += expected_dim[0] * expected_dim[1] + expected_dim[0]
        if index * 2 + 1 < len(graph.node):
            activation = graph.node[index * 2 + 1]
            if list(activation.input) != list(node.output):
                raise ValueError(f"activation {activation.name!r} is not connected to {node.name!r}")
            if activation.op_type == "Elu":
                attributes = {a.name: onnx.helper.get_attribute_value(a) for a in activation.attribute}
                if attributes.get("alpha", 1.0) != 1.0:
                    raise ValueError(f"ELU {activation.name!r} has incompatible alpha")
            previous = activation.output[0]
        else:
            previous = node.output[0]
    if previous != outputs[0].name:
        raise ValueError("final activation is not connected to the declared output")
    if len(result) != expected_count * 4 or len(result) != spec["weight_bytes"]:
        raise ValueError(f"extracted {len(result)} bytes, expected {spec['weight_bytes']}")
    if not all(math.isfinite(value[0]) for value in struct.iter_unpack("<f", result)):
        raise ValueError("model weights contain NaN or infinity")
    return bytes(result), expected_count


def format_u64(data):
    if len(data) % 8:
        raise ValueError("weight blob is not aligned to uint64")
    words = [int.from_bytes(data[i : i + 8], "little") for i in range(0, len(data), 8)]
    lines = []
    for i in range(0, len(words), 8):
        lines.append("  " + ", ".join(f"0x{x:016x}U" for x in words[i : i + 8]) + ",")
    return "\n" + "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("onnx", type=Path)
    parser.add_argument("--target", choices=TARGETS, default="tjb_network")
    parser.add_argument("--output", type=Path, help="override output path, mainly for testing")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    model = onnx.load(str(args.onnx))
    blob, count = extract_weights(model, args.target)
    target = args.output or Path(__file__).resolve().parents[1] / "X-CUBE-AI" / "App" / f"{args.target}_data_params.c"
    original = target.read_bytes()
    text = original.decode("utf-8").replace("\r\n", "\n")
    match = array_pattern(args.target).search(text)
    if not match:
        raise SystemExit(f"weight array not found in {target}")
    declared = int(re.search(r"\[(\d+)\]", match.group(1)).group(1))
    if declared != count // 2:
        raise SystemExit(f"target declares {declared} uint64 words, model needs {count // 2}")
    existing = re.findall(r"0x([0-9a-fA-F]{16})U", match.group(2))
    if len(existing) != declared:
        raise SystemExit(f"target has {len(existing)} weight words, expected {declared}")
    existing_blob = b"".join(int(word, 16).to_bytes(8, "little") for word in existing)
    if args.dry_run:
        print(f"OK: {args.onnx} -> {count} FP32 parameters ({len(blob)} bytes); "
              f"{'unchanged' if blob == existing_blob else 'would update'}; no file written")
        return
    if blob == existing_blob:
        print(f"Unchanged: {target} already contains the weights from {args.onnx}")
        return
    updated = text[: match.start(2)] + format_u64(blob) + text[match.end(2) :]
    newline = "\r\n" if b"\r\n" in original else "\n"
    target.write_bytes(updated.replace("\n", newline).encode("utf-8"))
    print(f"Updated {target}: {count} FP32 parameters ({len(blob)} bytes) from {args.onnx}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as exc:
        raise SystemExit(f"ERROR: {exc}") from exc
