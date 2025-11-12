#include <vector>
#include <chrono>
#include <iostream>

class ExpensiveToCopy {
public:
    ExpensiveToCopy(int value) : data(1000, value) {} // 大量数据
    
    ExpensiveToCopy(const ExpensiveToCopy& other) : data(other.data) {
        // std::cout << "Expensive copy!" << std::endl;
    }
    
    ExpensiveToCopy(ExpensiveToCopy&& other) noexcept : data(std::move(other.data)) {
        // std::cout << "Move operation!" << std::endl;
    }

private:
    std::vector<int> data;
};

void performance_comparison() {
    const int iterations = 1000;
    
    // push_back测试
    auto start1 = std::chrono::high_resolution_clock::now();
    {
        std::vector<ExpensiveToCopy> vec;
        vec.reserve(iterations);
        for (int i = 0; i < iterations; ++i) {
            vec.push_back(ExpensiveToCopy(i)); // 构造临时 + 移动
        }
    }
    auto end1 = std::chrono::high_resolution_clock::now();
    
    // emplace_back测试  
    auto start2 = std::chrono::high_resolution_clock::now();
    {
        std::vector<ExpensiveToCopy> vec;
        vec.reserve(iterations);
        for (int i = 0; i < iterations; ++i) {
            vec.emplace_back(i); // 直接构造
        }
    }
    auto end2 = std::chrono::high_resolution_clock::now();
    
    auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(end1 - start1);
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2);
    
    std::cout << "push_back time: " << duration1.count() << " μs\n";
    std::cout << "emplace_back time: " << duration2.count() << " μs\n";
}

int main(int argc, char const *argv[]) {
    performance_comparison();

    return 0;
}