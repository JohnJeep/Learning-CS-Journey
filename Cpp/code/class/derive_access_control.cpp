/*
 * @Baseuthor: JohnJeep
 * @Date: 2020-06-08 10:44:52
 * @LastEditTime: 2021-09-09 22:25:49
 * @LastEditors: Windows10
 * @Description: 派生类访问控制权限分析
 *  https://www.programiz.com/cpp-programming/public-protected-private-inheritance#:~:text=protected%20inheritance%20makes%20the%20public%20and%20protected%20members,base%20class%20are%20inaccessible%20to%20the%20derived%20class.
 * https://stackoverflow.com/questions/860339/difference-between-private-public-and-protected-inheritance
 */
#include <iostream>
using namespace std;

class Base {
private:
    int m_a = 1;

protected:
    int m_b = 2;

public:
    int m_c = 3;

public:
    Base() {}
    ~Base() {}

	// function to access private member
	int getPrivate() {
		return m_a;
	}
};

class PublicDerived : public Base {
private:
    /* data */
public:
    void showPublic() {
        cout << "show PublicDerived class" << endl;
        // cout << "m_a:" << m_a << endl;  // m_a is private，子类中不能直接访问
        cout << "m_b:" << m_b << endl;     // m_b is protected
        cout << "m_c:" << m_c << endl;     // m_c is public
    }
    PublicDerived() {}
    ~PublicDerived() {}
	
    // main 函数中类的对象不能直接访问 protected 和 private 属性的成员，需封装成函数来间接访问
	// public function 去访问父类中 protected member，此时子类中 m_b is protected
	int getProtect() {
		return m_b;
	}

    // getPrivate() 属性变为 Public
};

class ProtectedDerived : protected Base {
private:
    /* data */
public:
    void showProtected() {
        cout << "show ProtectedDerived class" << endl;
        // cout << "m_a:" << m_a << endl;  // m_a is private，子类中不能直接访问
        cout << "m_b:" << m_b << endl; // m_b is protected
        cout << "m_c:" << m_c << endl; // m_c is protected
    }
    ProtectedDerived() {}
    ~ProtectedDerived() {}

	// public function 去访问父类中 protected member，此时子类中 m_b is protected
    int getProtected() {
        return m_b;
    }

	// public function 去访问父类中 public member，此时子类中 m_c is protected
    int getPublic() {
        return m_c;
    }

    // getPrivate() 属性变为 protected
};

class PrivateDerived : private Base {
private:
    /* data */
public:
    void showPrivate() {
        cout << "show PrivateDerived class" << endl;
        // cout << "m_a:" << m_a << endl;  // m_a is private，子类中不能直接访问
        cout << "m_b:" << m_b << endl;     // m_b is private
        cout << "m_c:" << m_c << endl;     // m_c is private
    }
    PrivateDerived() {}
    ~PrivateDerived() {}

	// public function 去访问父类中 protected member，此时子类中 m_b is private
    int getProtected() {
        return m_b;
    }

	// public function 去访问父类中 public member，此时子类中 m_c is private
    int getPublic() {
        return m_c;
    }

    // getPrivate() 属性变为 private
};

int main()
{
    PublicDerived object1;
    object1.showPublic();
    cout << "PublicDerived: private = " << object1.getPrivate() << endl;
    cout << "PublicDerived: protected = " << object1.getProtect() << endl;
    cout << "PublicDerived: public = " << object1.m_c << endl;         // 类外部直接调用子类 public 属性成员变量

    ProtectedDerived object2;
    object2.showProtected();
    // cout << "ProtectedDerived: private = " << object2.getPrivate() << endl; // 类外部不能直接访问 protected 属性成员
    cout << "ProtectedDerived: protected = " << object2.getProtected() << endl;
    cout << "ProtectedDerived: public = " << object2.getPublic() << endl;

    PrivateDerived object3;
    object3.showPrivate();
    // cout << "PrivateDerived: Private = " << object3.getPrivate() << endl; // 类外部不能直接访问 Private 属性成员
    cout << "PrivateDerived: protected = " << object3.getProtected() << endl;
    cout << "PrivateDerived: public = " << object3.getPublic() << endl;

    return 0;
}
