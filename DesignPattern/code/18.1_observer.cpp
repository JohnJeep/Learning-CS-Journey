/*
 * @Author: JohnJeep
 * @Date: 2021-01-19 15:32:38
 * @LastEditTime: 2021-01-19 17:58:29
 * @LastEditors: Please set LastEditors
 * @Description: Oberver模式再次探究
 */
#include <iostream>
#include <cstring>
#include <list>
using namespace std;

// abstract obsrver
class IObserver
{
public:
    IObserver(){};
    virtual ~IObserver(){};
    virtual void Update(const string& message_from_subject) = 0;
};

// abstract subject
class ISubject
{
private:
    /* data */
public:
    ISubject(/* args */) {}
    virtual ~ISubject() {}
    virtual void Attach(IObserver* observer) = 0;
    virtual void Detach(IObserver* observer) = 0;
    virtual void Notify() = 0;
};

// concrete subject
class Subject : public ISubject
{
public:
    Subject(/* args */) {}
    virtual ~Subject() { cout << "Goodbye, I was the Subject." << endl; }

    /**
     * The subscription management methods.
     */
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
        Notify();
    }

    void HowManyObserver() 
    {
        cout << "There are " << m_list_observer.size() 
             << " observers in the list." << endl;
    }

    void Notify() override
    {
        list<IObserver*> ::iterator iterator = m_list_observer.begin();
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
    void SomeBuinessLogic()    // no use
    {
        this->m_message = "change message";
        Notify();
        cout << "I'm about to do something important." << endl;
    }

private:
    list<IObserver*> m_list_observer;
    string m_message;
};

// concrete oberver
class Observer : public IObserver
{
public:
    Observer(Subject& subject)
        :m_subject(subject) 
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
        m_subject.Detach(this);    // this: 运行时，解析的是哪个就传入的是哪个
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
    Observer* obser1 = new Observer(*sub);
    Observer* obser2 = new Observer(*sub);
    Observer* obser3 = new Observer(*sub);
    Observer* obser4;
    Observer* obser5;

    sub->CreateMessage("Hello: D");
    obser3->RemoveMeFromList();

    sub->CreateMessage("The weather is hot today! :p");
    obser4 = new Observer(*sub);

    obser2->RemoveMeFromList();
    obser5 = new Observer(*sub);

    sub->CreateMessage("My new car is great!");
    obser5->RemoveMeFromList();

    obser4->RemoveMeFromList();
    obser1->RemoveMeFromList();

    delete obser1;
    delete obser2;
    delete obser3;
    delete obser4;
    delete obser5;
}

using namespace std;

int main() 
{
    ClientCode();

    return 0;
}