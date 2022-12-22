不同目录下构建自己创建的不同 package

1. 项目对应的目录下执行 `go mod init project_name`，创建一个 `go.mod` 文件，这种方式需要在
   环境变量中配置 `GO111MODULE=on`，表明使用 go mod 方式去进行构建。
   
   > 开启模块后，`$GOPATH` 不再被用于解析包的导入， 也就是 go tool 不会从 GOPATH 中寻找应的包

2. 执行 `go mod tidy`
3. 不同的 package 创建不同的文件，文件名通常与 package 的名字一样
4. 分别在不同的 package 下编写代码
5. 在 main 包中导入自己创建的 package，采用 `import` 方式导入包的路径
6. 在 main 包的路径下编译，将 main 包编译成一个可执行的文件。`go build main.go`