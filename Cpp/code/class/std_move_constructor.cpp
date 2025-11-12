#include <iostream>
#include <string>
#include <vector>
#include <deque>
using  namespace std;

class Stu
{
public:
    Stu() // default constructor
    {
        cout << "default constructor called" << endl;
    }
    Stu(string name) // parameterized constructor
    {
        cout << "parameterized constructor called" << endl;
        this->name = name;
    } 
    Stu(const Stu& other) // copy constructor
    {
        cout << "copy constructor called" << endl;
        this->name = other.name;
    } 
    Stu& operator=(const Stu& other) // copy assignment operator
    {
        cout << "copy assignment operator called" << endl;
        if (this != &other) {
            this->name = other.name;
        }
        return *this;
    }
    Stu(Stu&& other) noexcept // move constructor
    {
        cout << "move constructor called" << endl;
        this->name = other.name;
    }
    Stu& operator=(Stu&& other) noexcept // move assignment operator
    {
        cout << "move assignment operator called" << endl;
        if (this != &other) {
            this->name = other.name;
        }
        return *this;
    }

    ~Stu() // destructor
    {
        cout << "destructor called for " << name << endl;
    }

private:
    string name;
};

Stu createStudent() {
    return Stu("Temporary");
}

void lvalue_rlvaue_test()
{
    int a = 100;
    int& ref = a;
    a = 11;
    cout << "ref: " << ref << endl;

    int&& rref = 200;
    cout << "rref: " << rref << endl;
}

int main(int argc, char const *argv[])
{
    // lvalue_rlvaue_test();

    Stu s1; // default constructor
    Stu s2("Bob"); // parameterized constructor
    Stu s3 = s2; // copy constructor
    Stu s4(s2); // other useage: copy constructor  
    Stu s5;
    s5 = s2; // copy assignment operator

    Stu s6 = std::move(s2); // move constructor
    Stu s7(Stu("Charlie")); // move constructor from temporary object
    
    Stu s9 = createStudent(); // move constructor from function return value
    Stu s10;
    s10 = std::move(s7); // move assignment operator

    // dynamic create object 
    cout << "\nDynamic allocation:" << endl;
    Stu* ps1 = new Stu("DynamicStu"); // parameterized constructor
    Stu* ps2 = new Stu(); // default constructor
    delete ps1; // destructor
    delete ps2; // destructor

    // using STL container
    cout << "\nUsing vector container:" << endl;
    vector<Stu> vec;
    vec.push_back(Stu("VectorStu1")); // move constructor
    vec.emplace_back("VectorStu2"); // parameterized constructor
    vec.push_back(s3); // copy constructor
    vec.push_back(std::move(s4)); // move constructor
    cout << endl;
    
    return 0;
}
