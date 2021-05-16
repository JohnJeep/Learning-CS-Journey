#ifndef INTEGER_H_
#define INTEGER_H_

class Integer
{
private:
    int m_value;

public:
    Integer();
    Integer(int value); 
    ~Integer();

    Integer operator+(Integer other); // 声明operator+，重载 operator+ 操作符
    int IntValue();
};

#endif /*INTEGER_H_ */
