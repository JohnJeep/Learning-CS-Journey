<!--
 * @Author: JohnJeep
 * @Date: 2023-07-25 14:51:53
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-07-25 15:33:23
 * @Description: Linux 下常用信号解释
 * Copyright (c) 2023 by John Jeep, All Rights Reserved. 
-->


# 1. Linux 下常用信号解释

Linux 终端下输入 `man 7 signal` 命令，可查看信号的使用手册。

```sh
$man 7 signal
SIGNAL(7)                 Linux Programmer's Manual                 SIGNAL(7)

NAME
       signal - overview of signals

DESCRIPTION
       Linux supports both POSIX reliable signals (hereinafter "standard sig‐
       nals") and POSIX real-time signals.

   Signal dispositions
       Each signal has  a  current  disposition,  which  determines  how  the
       process behaves when it is delivered the signal.

       The  entries  in  the  "Action" column of the tables below specify the
       default disposition for each signal, as follows:

       Term   Default action is to terminate the process.

       Ign    Default action is to ignore the signal.

       Core   Default action is to terminate the process and dump  core  (see
              core(5)).

       Stop   Default action is to stop the process.

       Cont   Default  action  is  to continue the process if it is currently
              stopped.
			......
		       First the signals described in the original POSIX.1-1990 standard.

       Signal     Value     Action   Comment
       ──────────────────────────────────────────────────────────────────────
       SIGHUP        1       Term    Hangup detected on controlling terminal
                                     or death of controlling process
       SIGINT        2       Term    Interrupt from keyboard
       SIGQUIT       3       Core    Quit from keyboard
       SIGILL        4       Core    Illegal Instruction
       SIGABRT       6       Core    Abort signal from abort(3)
       SIGFPE        8       Core    Floating point exception
       SIGKILL       9       Term    Kill signal
       SIGSEGV      11       Core    Invalid memory reference
       SIGPIPE      13       Term    Broken pipe: write to pipe with no
                                     readers
       SIGALRM      14       Term    Timer signal from alarm(2)
       SIGTERM      15       Term    Termination signal
       SIGUSR1   30,10,16    Term    User-defined signal 1
       SIGUSR2   31,12,17    Term    User-defined signal 2
       SIGCHLD   20,17,18    Ign     Child stopped or terminated
       SIGCONT   19,18,25    Cont    Continue if stopped
       SIGSTOP   17,19,23    Stop    Stop process
       SIGTSTP   18,20,24    Stop    Stop typed at terminal
       SIGTTIN   21,21,26    Stop    Terminal input for background process
       SIGTTOU   22,22,27    Stop    Terminal output for background process

       The signals SIGKILL and SIGSTOP cannot be caught, blocked, or ignored.

```

## 1.1. SIGINT
SIGINT 是一个重要的操作系统信号，它表示"中断"信号（Interrupt Signal）。`SIGINT` 信号通常由终端用户通过在终端中按下 `Ctrl+C` 组合键来发送给正在运行的程序。

`SIGINT` 信号的主要用途是 请求进程中断当前操作并终止执行。当用户想要停止一个正在运行的程序时，可以通过发送 `SIGINT` 信号来中断该程序的执行。

一些常见的用例和行为：

1. **终止程序：** 当用户在终端中按下 `Ctrl+C` 时，终端会向正在运行的程序发送 `SIGINT` 信号，请求它中止执行。程序可以捕捉 `SIGINT` 信号，并根据需要执行终止处理程序，例如关闭文件、保存数据或释放资源，然后正常退出。
2. **强制终止：** 如果程序没有捕捉 `SIGINT` 信号或捕捉信号后没有正常退出，操作系统可以采取进一步措施来强制终止程序的执行。这时，操作系统会发送 `SIGINT` 信号的衍生信号 `SIGKILL`（也称为"杀死信号"），该信号会立即终止进程而无需进一步处理。


## 1.2. SIGTERM 
`SIGTERM` 是一个重要的操作系统信号，它表示"终止"或"终止信号"（Terminate Signal）。`SIGTERM` 信号用于请求进程正常终止，它向进程发送一个通知，告知进程需要关闭并退出。

与 `SIGKILL` 信号不同，`SIGTERM` 信号是一种优雅的终止方式，它允许进程在终止之前进行清理和释放资源。进程可以通过捕捉 `SIGTERM` 信号并执行特定的终止处理程序来实现优雅的退出。

一些常见的用例和行为：

1. **正常终止：** 当用户或系统管理员希望终止某个进程时，通常会发送 `SIGTERM` 信号给该进程。接收到 `SIGTERM` 信号的进程可以选择在退出之前完成未完成的操作，保存数据并释放资源。
2. **进程管理：** 在进程管理中，`SIGTERM` 信号是一个常见的方式来终止或关闭特定的进程。比如，当系统需要停止某个后台进程时，可以发送 `SIGTERM` 信号。
3. **优雅重启：** 在一些情况下，进程可能需要重新启动以加载新的配置或更新。在执行重启之前，通常会先发送 `SIGTERM` 信号给进程，等待它进行清理和关闭，然后再启动新的实例。

值得注意的是，尽管 `SIGTERM` 信号请求进程优雅地终止，但并不能保证进程一定会退出。进程有权选择是否响应 `SIGTERM` 信号。如果进程在一定时间内（通常是几秒钟）没有响应 `SIGTERM`信号，操作系统可能会发送 `SIGKILL` 信号给进程，这是一种强制终止的方式，会导致进程立即终止且无法进行清理工作


## 1.3. SIGHUP
`SIGHUP` 是一个表示“挂起”或“终端挂起”的信号。它是 Unix 和类 Unix 系统中的一个操作系统信号。在类 Unix 系统中，进程（包括后台进程）可以接收不同类型的信号，以通知它们发生的某些事件或请求执行某些操作。

`SIGHUP` 信号通常与终端相关联，当用户从终端注销（退出登录）时，操作系统会向终端关联的所有进程发送 `SIGHUP` 信号。这意味着进程应该在接收到 `SIGHUP` 信号时执行某种操作，通常是关闭或重新初始化。

主要用途：

1. 终端关闭时，发送 `SIGHUP` 信号给相关进程，以确保这些进程在终端会话结束时正确退出或重新初始化。
2. 在某些情况下，`SIGHUP` 信号还可以用作某些进程的重新加载配置文件的信号，使它们在配置文件更改后可以重新读取新的配置。

在某些情况下，进程可以忽略 `SIGHUP` 信号，或者通过捕捉信号并执行特定操作来处理它。具体处理方式取决于进程的设计和用途。