/*
 * @Author: JohnJeep
 * @Date: 2023-01-06 17:48:37
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-07-11 14:27:31
 * @Description: 继承是通过 匿名字段来实现的
 * Copyright (c) 2023 by John Jeep, All Rights Reserved. 
 */

package main

import (
	"fmt"
)

type Goods struct {
	Name  string
	Price float64
}

// 结构体中嵌入单个匿名结构体
type Books struct {
	// Goods // 嵌套的匿名结构体
	Color string
	// int   // 匿名字段是基础类型
}

// 结构体中嵌入多个匿名结构体，相当于多重继承
type EnglishBook struct {
	Goods // 匿名结构体
	Books
	Vocabulary int
}

// 结构体中嵌套有名结构体
type ChineseBook struct {
	g     Goods // 有名结构体
	b     Books
	Count int
}

// 结构体中嵌套指针
type MathBook struct {
	*Goods
	*Books
	Level int
}

func main() {
	fourthLevel := EnglishBook{
		Goods{
			Name:  "groceries",
			Price: 38.9,
		},

		Books{
			Color: "pink",
		},
		4986, // 按照结构体字面值来赋值
	}
	fmt.Println("English book:", fourthLevel.Name, fourthLevel.Price, fourthLevel.Color, fourthLevel.Vocabulary) // 省略了匿名结构体名字
	fmt.Println("Book2:", fourthLevel.Books.Color, fourthLevel.Goods.Name, fourthLevel.Goods.Price)

	primary := ChineseBook{Goods{"tools", 99.9}, Books{"White"}, 8989} // 初始化时按照结构体中定义的顺序初始化
	// 打印有名结构体
	fmt.Printf("Chinese Book: name: %s, price: %f, color: %s, count: %d\n",
		primary.g.Name, primary.g.Price, primary.b.Color, primary.Count)

	advance := MathBook{
		&Goods{ // 取地址
			Name:  "tools",
			Price: 189.6,
		},
		&Books{
			Color: "green",
		},
		4,
	}
	fmt.Printf("Math Book: name: %s, price: %f, color: %s, level: %d\n",
		advance.Name, advance.Price, advance.Color, advance.Level)
}
