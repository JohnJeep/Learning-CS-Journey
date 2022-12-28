package pro

import "fmt"

type Product struct {
	Name  string
	Price float32
	Count int
}

type Person struct {
	Name string
}

func (p *Person) Buy(pro1 *Product, pro2 *Product) {
	totalCount := pro1.Count + pro2.Count
	fmt.Printf("%s 买的总数：%d\n", p.Name, totalCount)

	allPrice := pro1.Price + pro2.Price
	fmt.Printf("%s 总花费：%f\n", p.Name, allPrice)
}
