/*
 * @Author: JohnJeep
 * @Date: 2020-08-13 11:12:26
 * @LastEditTime: 2021-05-25 20:44:04
 * @LastEditors: Please set LastEditors
 * @Description: 智能指针共享内存例子
 *               编译时需要加 -lrt 参数，需要在Linux下才能编译通过
 */
#include <memory>     // shared_ptr
#include <sys/mman.h> // shared memory
#include <fcntl.h>
#include <unistd.h>
#include <cstring>    // strerror
#include <cerrno>     // error
#include <cstring>
#include <iostream>

namespace shared_memory
{
    class SharedMemoryDetacher
    {
    private:
        /* data */
    public:
        SharedMemoryDetacher(/* args */) {}
        ~SharedMemoryDetacher() {}

        void operator() (int* p) 
        {
            cout << "unlink /temp1234" << endl;
            if (shm_unlink("/tmp1234") != 0) {
                std::cerr << "OOPs: shm_unlink() failed" << std::endl;
            }
        }
    };

    std::shared_ptr<int> getSharedIntMemory(int num)
    {
        void* mem;
        int shmfd = shm_open("/tmp1234", O_CREAT|O_RDWR, S_IRWXU|S_IRWXG);
        if (shmfd < 0) {
            throw std::string(strerror(errno));
        }
        if (ftruncate(shmfd, num*sizeof(int)) == -1) { 
            throw std::string(strerror(errno));
        }
        mem = mmap(nullptr, num*sizeof(int), PROT_READ|PROT_WRITE, MAP_SHARED, shmfd, 0);
        if (mem == MAP_FAILED) {
            throw std::string(strerror(errno));
        }
        return std::shared_ptr<int>(static_cast<int*>(mem), SharedMemoryDetacher());
    }
}


void test01()
{
    std::shared_ptr<int> smp(shared_memory::getSharedIntMemory(1000));

    for (int i = 0; i < 100; ++i) {
        smp.get()[i] = i*42; 
    }

    std::cout << "return" << std::endl;
    std::cin.get();

    // release shared memory here:
    smp.reset();
}

int main(int argc, char *argv[])
{
    test01();

    return 0;
}