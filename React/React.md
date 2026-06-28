基本概念

工具使用范围

```
TypeScript      →    让你写代码时更安全（可选）
    ↓
React           →    决定界面怎么构建（核心）
    ↓
Vite            →    把代码变成浏览器能跑的东西（工具）
```

三个工具**层次不同、互不替代**：

| 工具       | 层次     | 可以不用吗               |
| ---------- | -------- | ------------------------ |
| React      | UI 框架  | 不行，这是核心           |
| Vite       | 构建工具 | 可以换别的，但必须有一个 |
| TypeScript | 类型系统 | 完全可选，新手可以先不用 |



React 代码的格式叫 **JSX**（JavaScript XML），本质上是 **JavaScript 的语法扩展**。

JSX 就是**在 JavaScript 里直接写 HTML 结构**：

```jsx
// 普通 JavaScript
const element = React.createElement('h1', null, 'Hello World');

// JSX 写法（同样的意思，但可读性强多了）
const element = <h1>Hello World</h1>;
```

JSX 不是真的 HTML，Vite 会在背后把它转换成上面那种 `React.createElement(...)` 的形式，再交给浏览器运行。



文件类型的职责

```
.css   →  决定页面"长什么样"（颜色、间距、字体）
.tsx   →  决定页面"有什么"（组件结构 + 交互逻辑）
.ts    →  纯粹的业务逻辑（数据处理、API请求、工具函数）
```



一个典型项目的文件分布

```
src/
├── App.tsx              ← 组件，有 JSX
├── main.tsx             ← 入口，有 JSX
├── components/
│   ├── Button.tsx       ← 组件，有 JSX
│   └── Modal.tsx        ← 组件，有 JSX
├── utils/
│   └── format.ts        ← 工具函数，没有 JSX
├── api/
│   └── user.ts          ← 请求函数，没有 JSX
└── types/
    └── index.ts         ← 类型定义，没有 JSX
```

