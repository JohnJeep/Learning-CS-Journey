<!--
 * @Author: JohnJeep
 * @Date: 2023-11-27 11:11:51
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-03-19 16:16:26
 * @Description: Actor 模型研究
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

## Actor

### `context.go`

```go
// Context contains contextual information for actors
type Context interface {
	infoPart
	basePart
	messagePart
	senderPart
	receiverPart
	spawnerPart
	stopperPart
	extensionPart
}

type ExtensionContext interface {
	extensionPart
}

type SenderContext interface {
	infoPart
	senderPart
	messagePart
}

type ReceiverContext interface {
	infoPart
	receiverPart
	messagePart
	extensionPart
}

type SpawnerContext interface {
	infoPart
	spawnerPart
}
```



### `message.go`

```go
// Actor is the interface that defines the Receive method.
//
// Receive is sent messages to be processed from the mailbox associated with the instance of the actor
type Actor interface {
	Receive(c Context)
}
```

Receive 方法是 Actor 接口的核心，它处理所有发送给 actor 的消息。

Receive 方法是由 ProtoActor 框架自动调用的。当一个 Actor 收到消息时，框架会自动调用该 Actor 的 Receive 方法，并将消息作为参数传递。

**在 ProtoActor 的内部，这是通过消息调度器（dispatcher）和邮箱（mailbox）来实现的。当一个消息被发送到 Actor 时，它首先被放入 Actor 的邮箱。然后，调度器会从邮箱中取出消息，并调用相应 Actor 的 Receive 方法。**

因此，开发者不需要手动调用 Receive 方法，只需要实现它，定义 Actor 如何响应各种消息即可。



### `dispatcher.go` 

消息调度器

```go
type Dispatcher interface {
	Schedule(fn func())
	Throughput() int
}
```

### `timer.go`

TimerScheduler

```go
func (s *TimerScheduler) SendRepeatedly(initial, interval time.Duration, pid *actor.PID, message interface{}) CancelFunc {
	return startTimer(initial, interval, func() {
		s.ctx.Send(pid, message)
	})
}
```

上面的代码是一个TimerScheduler结构体中的方法`SendRepeatedly`的实现。这个方法的功能是创建一个定时器，定期地向指定的`PID`发送特定的消息。

让我解释一下这个方法的参数和功能：

- `initial`：表示定时器第一次触发的延迟时间。
- `interval`：表示定时器之后每次触发的间隔时间。
- `pid`：是一个指向特定Actor实例的`PID`（Process ID），消息将会发送给这个Actor。
- `message`：是要发送的消息内容。

这个方法的作用是在定时器到期时，使用`ctx.Send(pid, message)`向指定的Actor发送消息。定时器在创建后，会在经过`initial`时间段后触发第一次，之后每隔`interval`时间触发一次，每次触发都会向指定的Actor发送特定的消息。

返回值是一个`CancelFunc`类型的函数，它可以用来取消定时器。通过调用这个函数，你可以停止后续的定时触发。

这种定时器的功能通常用于在分布式系统中实现定期的任务调度、周期性的状态检查或者执行一些周期性的清理操作。它使得在Actor模型中实现定时任务变得非常方便和直观。







## protoc-gen-gograinv2

template.go 模板文件分析

template.go 文件是 ProtoActor 项目中的一部分，它用于生成 Go 语言的代码。这个文件主要包含一个字符串常量 code，这个常量是一个模板，用于生成服务的代码。

这个模板主要包含以下部分：

1. **包的导入和日志设置**：这部分代码导入了一些必要的包，并设置了日志的级别。

2. **服务工厂的设置和获取**：对于每一个服务，都生成了一个工厂函数的设置和获取方法。工厂函数用于生成服务的实例。

3. **GrainClient 的获取**：对于每一个服务，都生成了一个获取 GrainClient 的方法。GrainClient 是用于与服务进行通信的客户端。

4. **Kind 的获取**：对于每一个服务，都生成了一个获取 Kind 的方法。Kind 是用于在集群中注册服务的类型。

5. **服务接口的定义**：对于每一个服务，都生成了一个接口。这个接口定义了服务的所有方法。

6. **GrainClient 的定义和方法**：对于每一个服务，都生成了一个 GrainClient 的结构体和对应的方法。这些方法用于向服务发送请求。

7. **Actor 的定义和方法**：对于每一个服务，都生成了一个 Actor 的结构体和对应的方法。这些方法用于处理接收到的消息。

这个模板的主要作用是根据 protobuf 文件生成对应的 Go 语言代码，这些代码可以用于创建服务、处理请求等。