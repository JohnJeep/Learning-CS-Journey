print("hello word")

print("hello word", end="!!!")
print("+++++++++++++++++++++++++++++++")

# Positional arguments
def introduce(name, age):
  print(f"My name is {name} and I am {age} years old.")

introduce("Alice", 30)

# keyword-only arguments
def display_info(name, age):
  print(f"Name: {name}, Age : {age}")

display_info(age=25, name="John")

# Positional-only and keyword-only arguments
def student(name, /, age, *, grade):
  ''' This function displays student information.'''
  print(f"Name: {name}, Age: {age}, Grade: {grade}")

student("Alice", 20, grade=100)

# Default parameter values
def greet(date, greeting="Hello", name="World"):
  print(f"{date}, {greeting}, {name}!")

greet("Mon", name="Bob")
greet.desc = "this is a greeting description"
print(greet.desc)

# Function with type hints
def add(a: int, b: int) -> int:
  return a + b

result = add(1, 2)
print(f"1 + 2 = {result}")