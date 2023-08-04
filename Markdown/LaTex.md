<!--
 * @Author: JohnJeep
 * @Date: 2019-03-13 09:55:36
 * @LastEditTime: 2023-08-04 11:34:38
 * @LastEditors: JohnJeep
 * @Description: LaTeX语法使用规则
-->

<!-- TOC -->

- [1. LaTex](#1-latex)
  - [1.1. 插入公式](#11-插入公式)
  - [1.2. 上下标](#12-上下标)
  - [1.3. 指定字体](#13-指定字体)
  - [1.4. 和号和积分号](#14-和号和积分号)
  - [1.5. 集合运算符](#15-集合运算符)
  - [1.6. 对数运算符](#16-对数运算符)
  - [1.7. 三角运算符](#17-三角运算符)
  - [1.8. 微积分运算符](#18-微积分运算符)
  - [1.9. 逻辑运算符](#19-逻辑运算符)
  - [1.10. 戴帽符号](#110-戴帽符号)
  - [1.11. 箭头符号](#111-箭头符号)
  - [1.12. 连线符号](#112-连线符号)
- [2. References](#2-references)

<!-- /TOC -->

# 1. LaTex

## 1.1. 插入公式
LaTeX 公式有两种，一种是用在正文中的，一种是单独命令的。正文中的公式用`$...$` 来定义，单独显示的用 `$$...$$` 来定义，其中 `...` 表示的是LaTeX 的公式命令。

例如：

> 定义$f(x) = \sum_{i=0}^{N}\int_{a}^{b} g(t,i) \text{ d}t$. (行内公式)

> 定义$f(x)$如下（行间公式）: 
> $$f(x) = \sum_{i=0}^{N}\int_{a}^{b} g(t,i) \text{ d}t{6}\tag{1}$$

\iint \limits_{-\infty}^{+\infty}  $\Longrightarrow$  $\iint \limits_{-\infty}^{+\infty}$

\int_{-\infty}^{+\infty} \int_{-\infty}^{+\infty}f(x,y) \,dx\,dy $\Longrightarrow$ $\int_{-\infty}^{+\infty} \int_{-\infty}^{+\infty}f(x,y) \,dx\,dy$

\int_1^2 \int_3^4{x}dx $\Longrightarrow$  $\int_1^2 \int_3^4{x}dx$

\sqrt[3]{2}   $\Longrightarrow$ $\sqrt[3]{2}$



## 1.2. 上下标
` ^ `表示上标，`_`表示下标，如果上（下）标内容多于一个字符就需要使用`{}`，注意不是`( )`, 因为( )经常是公式本身组成部分，为避免冲突，所以选用了`{ }` 将其包起来

示例：$x^{y^z}=(1+e^x)^{-2xy^w}$


希腊字母
|命令	   |显示   |命令     |显示    |
| ---      | ---   | ---    | ---    |
|\alpha	   |α	   |\beta	|β       |
|\gamma	   |γ	   |\delta	|δ       |
|\epsilon  |ϵ      |\zeta	|ζ       |
|\eta	   |η	   |\theta	|θ       |
|\iota	   |ι	   |\kappa	|κ       |
|\lambda   |λ      |\mu	    |μ       |
|\xi	   |ξ 	   |\nu     |ν       |
|\pi	   |π	   |\rho	|ρ       |
|\sigma	   |σ	   |\tau	|τ       |
|\upsilon  |υ      |\phi    |ϕ       |
|\chi	   |χ	   |\psi    |ψ       |
|\omega	   |ω	   |\Omega	|$\omega$|

- 如果使用大写的希腊字母，把命令中的首字母变成大写即可，例如 `\Gamma` 输出的是 $\Gamma$
- 如果使用斜体大写希腊字母，再在大写希腊字母的LaTeX命令前加上`var`，例如 $\Gamma$ 生成 $\varGamma$

例如：
$$
 \varGamma(x) = \frac{\int_{\alpha}^{\beta} g(t)(x-t)^2\text{ d}t }{\phi(x)\sum_{i=0}^{N-1} \omega_i} \tag{2}
$$


## 1.3. 指定字体
- {\rm text}如： 
- 使用罗马字体：text `${\rm text}$`
- 其他的字体还有： 
```
\rm　　罗马体　　　　　　　\it　　意大利体 
\bf　　黑体　　　　　　　　\cal 　花体 
\sl　　倾斜体　　　　　　　\sf　　等线体 
\mit 　数学斜体　　　　　　\tt　　打字机字体 
\sc　　小体大写字母
```


## 1.4. 和号和积分号
列出常用的和号和积分号

|显示                 |	命令             |
| ---                 | ---              |
|$\sum$               |\sum              |
|$\int$               |\int              |
|$\sum_{i=1}^{N}$     |\sum_{i=1}^{N}    |
|$\int_{a}^{b}$       |$int_{a}^{b}      |
|$\prod$              |\prod             |
|$\iint$              |\iint             |
|$\prod_{i=1}^{N}$	  |\prod_{i=1}^{N}   |
|$\iint_{a}^{b}$      |iint_{a}^{b}      |
|$\bigcup$            |\bigcup           |
|$\bigcap$            |\bigcap           |
|$\bigcup_{i=1}^{N}$  |\bigcup_{i=1}^{N} |
|$\bigcap_{i=1}^{N}$  |\bigcap_{i=1}^{N} |
|$\pm$                |\pm               |
|$\times$             |\times            |
|$\div$               |\div              |
|$\mid$               |\mid              |
|$\cdot$              |\cdot             |
|$\circ$              |\circ             |
|$\ast$               |\ast              |
|$\bigodot$           |\bigodot          |
|$\bigotimes$         |\bigotimes        |
|$\bigoplus$          |\bigoplus         |
|$\leq$               |\leq              |
|$\geq$               |\geq              |
|$\neq$               |\neq              |
|$\approx$            |\approx           |
|$\equiv$             |\equiv            |
|$\sum$               |\sum              |
|$\prod$              |\prod             |
|$\coprod$            |\coprod           |


## 1.5. 集合运算符
|显示	      |命令      |
| ---         | ---      |
|$\emptyset$  |\emptyset |
|$\in$        |\in       |
|$\notin$     |\notin    |
|$\subset$    |\subset   |
|$\supset$    |\supset   |
|$\subseteq$  |\subseteq |
|$\supseteq$  |\supseteq |
|$\bigcap$    |\bigcap   |
|$\bigcup$    |\bigcup   |
|$\bigvee$    |\bigvee   |
|$\bigwedge$  |\bigwedge |
|$\biguplus$  |\biguplus |
|$\bigsqcup$  |\bigsqcup |


## 1.6. 对数运算符 
|显示	|命令   |
| ---   | ---   |
|$\log$ |\log   |
|$\lg$  |\lg    |
|$\ln$  |\ln    |


## 1.7. 三角运算符
|显示	    |命令     |
| ---       | ---     |
|$\bot$     |\bot     |
|$\angle$   |\angle   |
|$30^\circ$ |30^\circ |
|$\sin$     |\sin     |
|$\cos$     |\cos     |
|$\tan$     |\tan     |
|$\cot$     |\cot     |
|$\sec$     |\sec     |
|$\csc$     |\csc     |


## 1.8. 微积分运算符
|显示	   |命令    |
| ---      | ---    |
|$\prime$  |\prime  |
|$\int$    |\int    |
|$\iint$   |\iint   |
|$\iiint$  |\iiint  |
|$\oint$   |\oint   |
|$\lim$    |\lim    |
|$\infty$  |\infty  |
|$\nabla$  |\nabla  |


## 1.9. 逻辑运算符
|显示	       |命令        |
| ---          | ---        |
|$\because$    |\because    |
|$\therefore$  |\therefore  |
|$\forall$     |\forall     |
|$\exists$     |\exists     |
|$\not=$       |\not=       |
|$\not>$       |\not>       |
|$\not\subset$ |\not\subset |


## 1.10. 戴帽符号
|显示	     |命令      |
| ---        | ---      |
|$\hat{y}$   |\hat{y}   |
|$\check{y}$ |\check{y} |
|$\breve{y}$ |\breve{y} |


## 1.11. 箭头符号
|显示	           |命令             |
| ---              | ---            |
|$\uparrow$        |\uparrow        |
|$\downarrow$      |\downarrow      |
|$\Uparrow$        |\Uparrow        |
|$\Downarrow$      |\Downarrow      |
|$\rightarrow$     |\rightarrow     |
|$\leftarrow$      |\leftarrow      |
|$\Rightarrow$     |\Rightarrow     |
|$\Leftarrow$      |\Leftarrow      |
|$\longrightarrow$ |\longrightarrow |
|$\longleftarrow$  |\longleftarrow  |
|$\Longrightarrow$ |\Longrightarrow |
|$\Longleftarrow$  |\Longleftarrow  |


## 1.12. 连线符号
|显示	                                        |命令              |
| ---                                           | ---              |
|$\overline{a+b+c+d}$                           |\overline{a+b+c}  |
|$\underline{a+b+c+d}$                          |\underline{a+b+c} |
|$\overbrace{a+\underbrace{b+c}_{1.0}+d}^{2.0}$ |\overbrace{a+\underbrace{b+c}_{1.0}+d}^{2.0}|



# 2. References
- [用Python学《微积分B》](https://blog.csdn.net/Sagittarius_Warrior/article/details/77671792)