#include <algorithm>
#include <iostream>
#include <memory>
#include <numeric>
#include <vector>

// 使用变量模板定义数学常数
template <typename T = double>
constexpr T pi = T(3.1415926535897932385);

// 使用constexpr函数进行编译时计算
constexpr double circle_area(double radius) { return pi<> * radius * radius; }

// 使用泛型Lambda和返回类型推导
template <typename T>
auto create_multiplier(T factor) {
  return [factor](auto value) { return value * factor; };
}

// 使用二进制字面量和数字分位符定义标志位
enum class Permissions : uint8_t {
  NONE = 0b0000,
  READ = 0b0001,
  WRITE = 0b0010,
  EXECUTE = 0b0100,
  ALL = 0b0111
};

// 为enum class重载按位或运算符
constexpr Permissions operator|(Permissions lhs, Permissions rhs) {
  return static_cast<Permissions>(
    static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs)
  );
}

class Shape {
 public:
  virtual ~Shape() = default;
  virtual double area() const = 0;
};

class Circle : public Shape {
 private:
  double radius_;

 public:
  constexpr Circle(double radius) : radius_(radius) {}

  constexpr double area() const override { return circle_area(radius_); }
};

void comprehensive_demo()
{
  // 使用make_unique创建对象
  auto shapes = std::vector<std::unique_ptr<Shape>>();
  shapes.push_back(std::make_unique<Circle>(5.0));
  shapes.push_back(std::make_unique<Circle>(10.0));

  // 使用泛型Lambda和算法
  auto multiplier = create_multiplier(2.5);
  std::vector<double> values{1.0, 2.0, 3.0, 4.0, 5.0};

  std::cout << "Original values: ";
  for (auto v : values) std::cout << v << " ";
  std::cout << std::endl;

  std::transform(values.begin(), values.end(), values.begin(), multiplier);

  std::cout << "After transformation: ";
  for (auto v : values) std::cout << v << " ";
  std::cout << std::endl;

  // 使用二进制标志
  Permissions user_perms = Permissions::READ | Permissions::WRITE;
  std::cout << "User permissions: " << static_cast<int>(user_perms)
            << std::endl;

  // 编译时计算
  constexpr double computed_area = circle_area(2.0);
  std::cout << "Compile-time computed area: " << computed_area << std::endl;
}

int main() 
{
  comprehensive_demo();
  return 0;
}