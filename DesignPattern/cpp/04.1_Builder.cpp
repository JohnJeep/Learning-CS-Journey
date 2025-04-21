/*
 * @Author: JohnJeep
 * @Date: 2020-12-23 21:10:27
 * @LastEditTime: 2022-07-08 10:57:54
 * @LastEditors: DESKTOP-0S33AUT
 * @Description: 重新理解Builder
 */
#include <cstring>
#include <iostream>
#include <stdlib.h>

using namespace std;

// Product
class Pizza {
public:
    void dough(const string& dough)
    {
        dough_ = dough;
    }

    void sauce(const string& sauce)
    {
        sauce_ = sauce;
    }

    void topping(const string& topping)
    {
        topping_ = topping;
    }

    void open() const
    {
        cout << "Pizza with " << dough_ << " dough, " << sauce_ << " sauce and "
             << topping_ << " topping. Mmm." << endl;
    }

private:
    string dough_;
    string sauce_;
    string topping_;
};

// Abstract Builder
class PizzaBuilder {
public:
    // Chinmay Mandal : This default constructor may not be required here
    PizzaBuilder()
    {
        // Chinmay Mandal : Wrong syntax
        // pizza_ = new Pizza;
    }
    const Pizza& pizza()
    {
        return pizza_;
    }

    virtual void buildDough() = 0;
    virtual void buildSauce() = 0;
    virtual void buildTopping() = 0;

protected:
    Pizza pizza_;
};

// concrete builder
class HawaiianPizzaBuilder : public PizzaBuilder {
public:
    void buildDough()
    {
        pizza_.dough("cross");
    }

    void buildSauce()
    {
        pizza_.sauce("mild");
    }

    void buildTopping()
    {
        pizza_.topping("ham+pineapple");
    }
};

// concrete builder
class SpicyPizzaBuilder : public PizzaBuilder {
public:
    void buildDough()
    {
        pizza_.dough("pan baked");
    }

    void buildSauce()
    {
        pizza_.sauce("hot");
    }

    void buildTopping()
    {
        pizza_.topping("pepperoni+salami");
    }
};

// Director
class Cook {
public:
    Cook()
        : pizzaBuilder_(NULL /*nullptr*/) // Chinmay Mandal : nullptr replaced with NULL.
    {
    }

    ~Cook()
    {
        if (pizzaBuilder_)
            delete pizzaBuilder_;
    }

    void pizzaBuilder(PizzaBuilder* pizzaBuilder)
    {
        if (pizzaBuilder_)
            delete pizzaBuilder_;

        pizzaBuilder_ = pizzaBuilder;
    }

    const Pizza& getPizza()
    {
        return pizzaBuilder_->pizza();
    }

    void constructPizza()
    {
        pizzaBuilder_->buildDough();
        pizzaBuilder_->buildSauce();
        pizzaBuilder_->buildTopping();
    }

private:
    PizzaBuilder* pizzaBuilder_;
};

int main()
{
    Cook cook;
    cook.pizzaBuilder(new HawaiianPizzaBuilder);
    cook.constructPizza();

    Pizza hawaiian = cook.getPizza();
    hawaiian.open();

    cook.pizzaBuilder(new SpicyPizzaBuilder);
    cook.constructPizza();

    Pizza spicy = cook.getPizza();
    spicy.open();
}
