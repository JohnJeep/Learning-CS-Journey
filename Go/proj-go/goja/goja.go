// goja 学习

package main

import "fmt"

func main() {
	vm := joja.new()

	m := map[string]interface{}{}
	vm.Set("m", m)
	vm.RunString(`
	var obj = {test : false};
	m.obj = obj;
	obj.test = true;
	`)

	fmt.Println(m["obj"].(map[string]interface{})["test"])
}
