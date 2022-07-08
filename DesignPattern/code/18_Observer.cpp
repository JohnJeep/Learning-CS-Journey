/*
 * @Author: JohnJeep
 * @Date: 2021-01-19 15:32:38
 * @LastEditTime: 2022-07-08 11:27:28
 * @LastEditors: DESKTOP-0S33AUT
 * @Description: Oberver 模式再次探究
 */
#include <cstring>
#include <iostream>
#include <list>
using namespace std;

// abstract obsrver
class IObserver {
public:
    IObserver() {};
    virtual ~IObserver() {};

    // 更新获得的观察目标信息
    virtual void Update(const string& message_from_subject) = 0;
};

// abstract subject
class ISubject {
public:
    ISubject() { }
    virtual ~ISubject() { }

    // 观察目标中添加观察者
    virtual void Attach(IObserver* observer) = 0;

    // 观察目标中删除观察者
    virtual void Detach(IObserver* observer) = 0;

    // 观察目标的状态有变化后，通知所有的观察者
    virtual void Notify() = 0;
};

// concrete subject
class Subject : public ISubject {
public:
    Subject() { }
    
    virtual ~Subject() 
    { 
        cout << "Goodbye, I was the Subject." << endl; 
    }

    void Attach(IObserver* observer) override
    {
        m_list_observer.push_back(observer);
    }

    void Detach(IObserver* observer) override
    {
        m_list_observer.remove(observer);
    }

    void CreateMessage(string message = "Empty")
    {
        this->m_message = message;
        Notify();   // 观察目标的状态发生变化后，通知所有已注册的观察者
    }

    void HowManyObserver()
    {
        cout << "There are " << m_list_observer.size()
             << " observers in the list." << endl;
    }

    void Notify() override
    {
        list<IObserver*>::iterator iterator = m_list_observer.begin();
        HowManyObserver();
        while (iterator != m_list_observer.end()) {
            (*iterator)->Update(m_message);
            ++iterator;
        }
    }

    /**
     * Usually, the subscription logic is only a fraction of what a Subject can
     * really do. Subjects commonly hold some important business logic, that
     * triggers a notification method whenever something important is about to
     * happen (or after it).
     */
    void SomeBuinessLogic() // no use
    {
        this->m_message = "change message";
        Notify();
        cout << "I'm about to do something important." << endl;
    }

private:
    list<IObserver*> m_list_observer;   // 待注册的观察者
    string m_message;
};

// concrete oberver
class Observer : public IObserver {
public:
    Observer(Subject& subject)
        : m_subject(subject)
    {
        this->m_subject.Attach(this);
        cout << "Hi, I'm the observer \"" << ++Observer::m_static_number
             << "\"." << endl;

        this->m_number = Observer::m_static_number;
    }
    virtual ~Observer()
    {
        cout << "Goodbye, I was the observer \"" << this->m_number << "\"" << endl;
    }

    void Update(const string& message_from_subject) override
    {
        m_message_from_subject = message_from_subject;
        PrintInfo();
    }

    void PrintInfo()
    {
        cout << "Observer \"" << this->m_number
             << "\" : a new message is available --> "
             << this->m_message_from_subject << endl;
    }

    void RemoveMeFromList()
    {
        m_subject.Detach(this); // this: 运行时，解析的是哪个就传入的是哪个
        cout << "Observer \"" << this->m_number << " \" remove from list." << endl;
    }

private:
    int m_number;
    Subject& m_subject;
    string m_message_from_subject;
    static int m_static_number;
};

// init static
int Observer::m_static_number = 0;

void ClientCode()
{
    Subject* sub = new Subject;
    Observer* observer1 = new Observer(*sub);
    Observer* observer2 = new Observer(*sub);
    Observer* observer3 = new Observer(*sub);
    Observer* observer4;
    Observer* observer5;

    sub->CreateMessage("Hello everyone!");
    observer3->RemoveMeFromList();

    sub->CreateMessage("The weather is hot today!");
    observer4 = new Observer(*sub);

    observer2->RemoveMeFromList();
    observer5 = new Observer(*sub);

    sub->CreateMessage("I'm very hot!");
    observer5->RemoveMeFromList();

    observer4->RemoveMeFromList();
    observer1->RemoveMeFromList();

    delete sub;
    delete observer1;
    delete observer2;
    delete observer3;
    delete observer4;
    delete observer5;
}

int main()
{
    ClientCode();

    return 0;
}