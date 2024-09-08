def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

value = 2
min_value = 5
max_value = 15

value = clamp(value, min_value, max_value)

print(value)  # Output will be 15
