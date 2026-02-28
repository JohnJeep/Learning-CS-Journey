class MyContext:
    def __enter__(self):
        print("进入上下文")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        print("退出上下文")
        # 返回 True 可以抑制异常（一般不建议）
        return False

with MyContext() as mc:
    print("在 with 块中")

layer = int(input("请输入层数:"))
if layer % 2 == 0:
    tag_id = 127
    print("Tag ID 是 127")
else:
    tag_id = 128
    print("Tag ID 是 128")

