import json

print("---JSON parsing---")
json_str = '{"name": "aa", "age": 30}'
data = json.loads(json_str)
print(data)
print(data["name"])
print(data.get("ag"))

print("---numeric operations---")
print(10/3)  # true division
print(10//3) # floor division
print(10%3)  # modulus

print("---math module---")
import math
print(math.sin(math.radians(30)))
print(math.sin(math.pi / 2))

print(math.radians(30))  # xxx度等于多少弧度
print(math.degrees(0.5)) # xxx弧度等于多少度


print("---list iteration---")
# iterate with index and value
words = ['apple', 'banana', 'cherry']
for index, word in enumerate(words):
    print(index, word)

# iterate only values
for word in words:
    print(word)

print("---dictionary iteration---")
# iterate key and value
maps = {'a': 1, 'b': 2, 'c': 3}
for key, value in maps.items():
    print(key, value)

# iterate only keys
for key in maps:
    print(key)

# iterate only values
for value in maps.values():
    print(value)

print("---file operations---")
import os
filename = "data.txt"
if not os.path.exists(filename):
    print(f"file {filename} does not exist, creating")
    with open(filename, "w") as f:
        f.write("Hello, World!\n")
else:
    print(f"file {filename} exists, reading")
    with open(filename, "r") as f:
        content = f.read()
        print(content)

