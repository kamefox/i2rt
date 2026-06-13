def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    return float(x_int) * span / float((1 << bits) - 1) + x_min
