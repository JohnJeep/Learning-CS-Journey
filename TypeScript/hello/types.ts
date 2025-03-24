// type declarations
let a: number;
let b: string;
let c: boolean;
let e: number[] = [1, 2, 3];
let i: number | string;
let f: Array<number> = [1, 2, 3];
let d: any;
let g: any[] = [1, "2", true];
let k: void;
let l: null;
let m: undefined;
let n: never;
let o: object;
let p: unknown;
let mytupe: [string, number] = ["hello", 10];
enum Direction {
    Up = 1,
    Down,
    Left,
    Right
}

a = 10;
b = "good";
console.log(a, b);
console.log(mytupe);
console.log(Direction.Up);

// 测试类型名称是大写与小写的区别
let t1 :string = "hello";
let t2 :String;
t2 = new String("hello");
console.log(typeof t1);
console.log(typeof t2);

let x : unknown;
x= 'unknown';
// 断言第一种写法
let y = x as string;

// 断言第二种写法
let z = <string>x;