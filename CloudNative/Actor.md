<!--
 * @Author: JohnJeep
 * @Date: 2023-09-15 11:03:37
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-04 19:26:49
 * @Description: 分布式 Actor 学习
 * Copyright (c) 2023 by John Jeep, All Rights Reserved. 
-->

# 1. Actor Model

## 1.1. 带着问题思考

### 1.1.1. Actors是如何发送和接收消息的？

在Actor模型中，消息是唯一的通信机制。Actors通过消息进行相互通信，它们不直接共享内存。以下是Actors如何发送和接收消息的基本概念：

1. **发送消息：**

Actors可以发送消息给其他Actors，也可以发送消息给自己。当一个Actor想要与另一个Actor通信时，它创建一个消息并将其发送给目标Actor的地址（通常称为PID，即Process ID）。

在发送消息时，Actor系统负责将消息传递给正确的接收者。发送消息通常是异步的，这意味着发送者Actor不会等待接收者Actor处理消息。发送消息的过程通常是非阻塞的，这样可以提高系统的并发性能。

2. **接收消息：**

当一个Actor接收到消息时，它会将消息放入自己的邮箱（Mailbox）中。邮箱是一个消息队列，用于存储Actor接收到的消息。接收者Actor从邮箱中获取消息，并根据消息的内容执行相应的操作。

接收消息的过程通常是同步的，即当Actor正在处理一个消息时，它会等待该消息处理完成，然后再处理下一个消息。这确保了消息的顺序处理，避免了竞态条件和数据一致性问题。

3. **消息处理：**

当Actor接收到消息后，它会调用预定义的消息处理函数（也称为处理器）来处理消息。处理器是Actor定义的一部分，用于指定接收到特定类型消息时应该执行的操作。处理器可以是Actor的方法，这样当Actor接收到特定类型的消息时，它会调用相应的方法来处理消息。

4. **消息传递的特性：**

在Actor模型中，消息传递具有以下特性：

- **封装性（Encapsulation）：** 每个Actor都有自己的状态和行为，其他Actor无法直接访问其状态，只能通过消息传递来与其通信。

- **并发性（Concurrency）：** 多个Actor可以并发地执行，处理各自的消息，从而实现系统级的并发性。

- **位置透明性（Location Transparency）：** 发送消息时，发送者无需知道接收者Actor的具体位置，只需知道接收者的PID即可。

  

### 1.1.2. 它们是如何并发地工作的？



### 1.1.3. 理解代码中的设计模式和结构



## 1.2. 模型介绍

Proto.Actor-go 是一个用于构建分布式应用程序的框架，它基于 Go 语言开发，旨在简化并发和分布式系统的开发。这个框架使用 Actor 模型作为基础，允许开发者将应用程序拆分成独立的 Actor，并通过消息传递进行通信。

Actor模型是一个概念模型，用于处理并发计算。Actor由3部分组成：状态（State）+行为（Behavior）+邮箱（Mailbox），State是指actor对象的变量信息，存在于actor之中，actor之间不共享内存数据，actor只会在接收到消息后，调用自己的方法改变自己的state，从而避免并发条件下的死锁等问题；Behavior是指actor的计算行为逻辑；邮箱建立actor之间的联系，一个actor发送消息后，接收消息的actor将消息放入邮箱中等待处理，邮箱内部通过队列实现，消息传递通过异步方式进行。

## 1.3. Actor

### 1.3.1. 特点

- Actor 之间是彼此隔离的，不会共享内存，只能通过邮箱方式去和对方打交道。

  > 邮箱（MailBox）：本质上是一个队列。每个Actor都有一个邮箱，邮箱接收并缓存其他Actor发过来的消息。

- 彼此之间异步发送消息和处理的。

- Actor一次只能**同步**处理一个消息，处理消息过程中，除了可以接收消息外不能做任何其他操作。

- 多个 Actor 之间通过发信息进行交流，actor与actor之间可以交流，但是不会产生数据竞争。

- 一个Actor可以响应消息、退出新Actor、改变内部状态、将消息发送到一个或多个Actor。

- Actor可能会堵塞自己但Actor不应该堵塞自己运行的线程

- 多个actor向同一actor发送消息，按照时间顺序投递进对方MailBox

- actor 能够根据到来的消息通过行为修改自己的状态，可以发送信息给其他actor，可以创建新的actor或者新的子actor。

- actor需要知道接收方的地址，需要知道将消息传递给谁

### 1.3.2. 缺点

- Actor 没有拒绝交流的能力。当消息到达时，必须对其进行处理，或者将其放入缓冲区，以便稍后处理。

### 1.3.3. Spawn(生成)

在Proto Actor中，**`Spawn`是用于创建新的Actor实例**的操作，并返回该 Actor 的 `PID`。当你想要创建一个新的Actor时，你会调用`Spawn`函数，并且提供一个`Props`对象作为参数，该对象描述了将要生成的Actor的行为。`Spawn`会返回一个`PID`（Process ID）——一个用于唯一标识Actor实例的标识符。

```go
pid, err := actor.Spawn(props) // 通过Props对象生成一个新的Actor实例，并返回其PID
```

### 1.3.4. Props(属性)

`Props`是一个用于配置Actor实例的对象。它定义了Actor的行为、状态和消息处理逻辑。当你调用`Spawn`函数时，你需要提供一个`Props`对象，以告诉Proto Actor系统你希望创建的Actor的特性。

`Props`对象通常包含以下信息：

- **Actor类型（Actor Type）：** 描述Actor的类型，指定Actor实例所属的类型。
- **消息处理器（Message Handler）：** 定义了Actor如何处理接收到的消息。
- **监督策略（Supervision Strategy）：** 定义了当Actor失败时，如何处理该Actor的策略。
- **初始化参数（Initialization Parameters）：** 用于在Actor实例创建时传递给Actor构造函数的参数。

在Proto Actor中，`Props`对象可以按需定制，以创建不同类型的Actor实例。以下是一个示例：

```go
props := actor.PropsFromFunc(func(ctx actor.Context) {
    // 消息处理逻辑
    switch msg := ctx.Message().(type) {
    case string:
        ctx.Respond("Received: " + msg)
    }
})

pid, err := actor.Spawn(props)
```

在上述示例中，`PropsFromFunc`函数创建了一个`Props`对象，它指定了一个简单的消息处理器，该处理器能够处理接收到的字符串消息并回复。



### 1.3.5. state

ctor本身的属性信息，state只能被actor自己操作，不能被其他actor共享和操作，有效的避免加锁和数据竞争

### 1.3.6. behavior

Actor的处理逻辑。定义在该时间点对消息作出反应时要采取的行动的功能，例如，如果客户端得到授权，则转发请求，否则拒绝请求。此行为可能会随着时间的推移而改变

### 1.3.7. mailbox

- actor 存储消息是 fifo队列，每个actor只有一个信箱，所有发送者都将他们的消息队列排队。

- actor与actor之间发送消息，消息只能发送到邮箱，等待拥有邮箱的actor 去处理，这个过程是异步的。

### 1.3.8. PID

- 创建一个 Actor 时，不能直接向另一个 Actor 发送者消息，而是用开发者暴露出消息传递的一个引用（reference），这个引用就叫 `PID`，是 `process id`的简称。
- PID 是一个序列化标识，被使用发送 messages 到 Actor 的 mailbox。

- 通过 PID 就知道了实际 actor instance 位于哪儿以及是怎样通信的。

### 1.3.9. children

### 1.3.10. supervisor Strategy

- 每个actor都有一个监督者。
- Proto.Actor实现了一种称为“家长监督”的特定形式。actor只能由其他actor创建，其中顶级actor由库提供，每个创建的actor由其父级监控。
- 两类监管策略
  - ForOneStrategy：将获得的指令应用于失败的子级，没有明确指定，这就是默认值。
  - AllForOneStrategy：适用于child actor 之间存在紧密依赖关系的情况，即一个child的失败会影响其他child的功能，他们之间存在着不可分割的联系。

### 1.3.11. messages

- Actor 可以运行在本地进程，也可以在不同机器的远程Actor上运行。
- 消息是不可变的，因此是线程安全的。
- 消息是异步的，



### 1.3.12. 用法

Actor是ProtoActor的核心组件，它是一个可以处理消息的实体。每个Actor都有一个唯一的PID（Process ID）来标识。Actor通过消息进行通信，不共享状态，因此可以避免并发编程中的许多问题。

ReceiveDefault 是 UserActor 接口中定义的一个方法。在 ProtoActor 的设计中，这个方法的主要作用是处理 Actor 接收到的未被明确处理的消息。

在 Actor 模型中，Actor 通过消息进行通信。当 Actor 接收到一个消息时，它会在其 Receive 方法中查找对应的处理逻辑。如果 Receive 方法中没有找到对应的处理逻辑，那么这个消息就会被传递给 ReceiveDefault 方法。

```go
package main

import (
	"fmt"
	"time"

	console "github.com/asynkron/goconsole"
	"github.com/asynkron/protoactor-go/actor"
)

type (
	hello      struct{ Who string }
	helloActor struct{}
)

func (state *helloActor) Receive(context actor.Context) {
	switch msg := context.Message().(type) {
	case *actor.Started:
		fmt.Println("Started, initialize actor here")
	case *actor.Stopping:
		fmt.Println("Stopping, actor is about shut down")
	case *actor.Stopped:
		fmt.Println("Stopped, actor and its children are stopped")
	case *actor.Restarting:
		fmt.Println("Restarting, actor is about restart")
	case *hello:
		fmt.Printf("Hello %v\n", msg.Who)
	}
}

func main() {
	system := actor.NewActorSystem()
	props := actor.PropsFromProducer(func() actor.Actor { return &helloActor{} })
	pid := system.Root.Spawn(props)
	system.Root.Send(pid, &hello{Who: "Roger"})

	// why wait?
	// Stop is a system message and is not processed through the user message mailbox
	// thus, it will be handled _before_ any user message
	// we only do this to show the correct order of events in the console
	time.Sleep(1 * time.Second)
	system.Root.Stop(pid)

	_, _ = console.ReadLine()
}

```

### 1.3.13. Router

Router 是用于管理一组Actor并将消息路由到这些 Actor 的组件。ProtoActor提供了几种路由策略，如RoundRobin（轮询）、Random（随机）等。

```go
package main

import (
	"log"

	console "github.com/asynkron/goconsole"
	"github.com/asynkron/protoactor-go/actor"
	"github.com/asynkron/protoactor-go/router"
)

type workItem struct{ i int }

const maxConcurrency = 5

func doWork(ctx actor.Context) {
	if msg, ok := ctx.Message().(*workItem); ok {
		// this is guaranteed to only execute with a max concurrency level of `maxConcurrency`
		log.Printf("%v got message %d", ctx.Self(), msg.i)
	}
}

func main() {
	system := actor.NewActorSystem()
	pid := system.Root.Spawn(router.NewRoundRobinPool(maxConcurrency).Configure(actor.WithFunc(doWork)))
	for i := 0; i < 1000; i++ {
		system.Root.Send(pid, &workItem{i})
	}
	_, _ = console.ReadLine()
}

```

### 1.3.14. Persistence of actor's state

文献：Proto.Persistence module: https://proto.actor/docs/persistence-proto-persistence/

有场景需要存储 actor 的 state 时，可用 protoactor-go 中 `persistence` 模块的 events（事件） 和 snapshots（快照）。

## 1.4. Remote

Remote 是ProtoActor的分布式组件，它允许Actor跨网络节点进行通信。你可以创建一个Remote Actor，然后在另一个节点上通过PID发送消息给它。

学习 remote 可以清楚知道 actor 之间是如何通信的。

```go
package main

import (
	"fmt"
	"log"
	"runtime"
	"sync"
	"time"

	"remoterouting/messages"

	console "github.com/asynkron/goconsole"
	"github.com/asynkron/protoactor-go/actor"
	"github.com/asynkron/protoactor-go/remote"
	"github.com/asynkron/protoactor-go/router"
)

var (
	system      = actor.NewActorSystem()
	rootContext = system.Root
)

func main() {
	cfg := remote.Configure("127.0.0.1", 8100)
	r := remote.NewRemote(system, cfg)
	r.Start()

	runtime.GOMAXPROCS(runtime.NumCPU())
	runtime.GC()

	p1 := actor.NewPID("127.0.0.1:8101", "remote")
	p2 := actor.NewPID("127.0.0.1:8102", "remote")

	remotePID := rootContext.Spawn(router.NewConsistentHashGroup(p1, p2))

	messageCount := 1000000

	var wgStop sync.WaitGroup

	props := actor.
		PropsFromProducer(newLocalActor(&wgStop, messageCount),
			actor.WithMailbox(actor.Bounded(10000)))

	pid := rootContext.Spawn(props)

	log.Println("Starting to send")

	t := time.Now()

	for i := 0; i < messageCount; i++ {
		message := &messages.Ping{User: fmt.Sprintf("User_%d", i)}
		rootContext.RequestWithCustomSender(remotePID, message, pid)
	}

	wgStop.Wait()

	rootContext.Stop(pid)

	fmt.Printf("elapsed: %v\n", time.Since(t))

	console.ReadLine()
}

```

## 1.5. Scheduler

Scheduler是ProtoActor的定时任务组件，它可以让你在指定的时间后或者按照指定的间隔重复发送消息。

```go
package main

import (
	"log"
	"math/rand"
	"sync"
	"time"

	console "github.com/asynkron/goconsole"
	"github.com/asynkron/protoactor-go/actor"
	"github.com/asynkron/protoactor-go/scheduler"
)

var HelloMessages = []string{
	"Hello",
	"Bonjour",
	"Hola",
	"Zdravstvuyte",
	"Nǐn hǎo",
	"Salve",
	"Konnichiwa",
	"Olá",
}

func main() {
	var wg sync.WaitGroup
	wg.Add(5)

	rand.Seed(time.Now().UnixMicro())
	system := actor.NewActorSystem()

	log.SetFlags(log.LstdFlags | log.Lmicroseconds)

	count := 0
	props := actor.PropsFromFunc(func(ctx actor.Context) {
		switch t := ctx.Message().(type) {
		case []string:
			count++
			log.Printf("\t%s, counter value: %d", t[rand.Intn(len(t))], count)
			wg.Done()
		case string:
			log.Printf("\t%s\n", t)
		}
	})

	pid := system.Root.Spawn(props)

	s := scheduler.NewTimerScheduler(system.Root)
	cancel := s.SendRepeatedly(1*time.Millisecond, 1*time.Millisecond, pid, HelloMessages)

	wg.Wait()
	cancel()

	wg.Add(100) // add 100 to our waiting group
	cancel = s.RequestRepeatedly(1*time.Millisecond, 1*time.Millisecond, pid, HelloMessages)

	// the following timer will fire before the
	// wait group is consumed and will stop the scheduler
	time.Sleep(10 * time.Millisecond)
	cancel()

	s.SendOnce(1*time.Millisecond, pid, "Hello Once")

	// this message will never show as we cancel it before it can be fired
	cancel = s.RequestOnce(500*time.Millisecond, pid, "Hello Once Again")
	time.Sleep(250 * time.Millisecond)
	cancel()

	_, _ = console.ReadLine()
}

```

 **Plugin**：Plugin是ProtoActor的插件系统，你可以通过实现Plugin接口来创建自定义的插件。

```go
package plugin

import (
	"github.com/asynkron/protoactor-go/actor"
)

type plugin interface {
	OnStart(actor.ReceiverContext)
	OnOtherMessage(actor.ReceiverContext, *actor.MessageEnvelope)
}

func Use(plugin plugin) func(next actor.ReceiverFunc) actor.ReceiverFunc {
	return func(next actor.ReceiverFunc) actor.ReceiverFunc {
		fn := func(context actor.ReceiverContext, env *actor.MessageEnvelope) {
			switch env.Message.(type) {
			case *actor.Started:
				plugin.OnStart(context)
			default:
				plugin.OnOtherMessage(context, env)
			}

			next(context, env)
		}

		return fn
	}
}

```



## 1.6. Cluster

Proto.Actor的集群（Cluster）是一种分布式Actor系统，它允许在多个节点上运行Actor，并通过网络进行通信。集群的主要目标是提供高可用性和容错性。

以下是Proto.Actor集群的一些核心API：

1. **Cluster.Start**：这个函数用于启动一个集群节点。它需要一个ClusterConfig对象，该对象包含了集群的配置信息，如节点的地址、端口、种子节点列表等。

2. **Cluster.GetCluster**：这个函数返回一个Cluster对象，你可以通过这个对象进行集群的操作，如获取集群成员、注册或注销事件处理器等。

3. **Cluster.SpawnNamed**：这个函数用于在集群中创建一个Actor。它需要一个Props对象和一个名字，返回创建的Actor的PID。

4. **Cluster.PID**：这个函数用于获取集群中的一个Actor的PID。你可以通过这个PID向Actor发送消息。

以下是一个简单的使用Proto.Actor集群的例子：

```go
package main

import (
	"log"
	"time"

	"github.com/asynkron/protoactor-go/actor"
	"github.com/asynkron/protoactor-go/cluster"
	"github.com/asynkron/protoactor-go/remote"
)

type HelloActor struct{}

func (state *HelloActor) Receive(ctx actor.Context) {
	switch msg := ctx.Message().(type) {
	case *actor.Started:
		log.Println("Started, initialize actor here")
	case *actor.Stopping:
		log.Println("Stopping, actor is about shut down")
	case *actor.Stopped:
		log.Println("Stopped, actor and its children are stopped")
	case *actor.Restarting:
		log.Println("Restarting, actor is about restart")
	case string:
		log.Printf("Hello %v\n", msg)
	}
}

func main() {
	system := actor.NewActorSystem()
	remoteConfig := remote.Configure("localhost", 8080)
	remote := remote.NewRemote(system, remoteConfig)
	remote.Start()

	clusterConfig := cluster.Configure("mycluster", remote, nil)
	c := cluster.NewCluster(system, clusterConfig)
	c.Start("node1", "localhost:8080")

	props := actor.PropsFromProducer(func() actor.Actor { return &HelloActor{} })
	pid, err := c.SpawnNamed(props, "hello")
	if err != nil {
		log.Fatalf("Failed to spawn named actor: %v", err)
	}

	c.PID("hello").Tell("Hello World")

	time.Sleep(1 * time.Second)
	c.Shutdown(true)
}
```

在这个例子中，我们首先创建了一个Actor系统和一个远程配置，然后启动了一个远程节点。接着，我们创建了一个集群配置，并使用它启动了一个集群节点。然后，我们在集群中创建了一个名为"hello"的Actor，并向它发送了一个消息。最后，我们关闭了集群。



### 1.6.1. grain

在Proto.Actor中，Grain是一种特殊的Actor，它提供了一种更高级的抽象，使得开发者可以更加专注于业务逻辑，而不是并发和分布式计算的细节。

virtual actor model 借鉴了 `Microsoft Orleans` 的概念。一个`Grain` 就是一个 `virtual actors`

**Grain是一种虚拟Actor，它的生命周期由Proto.Actor框架自动管理。**Grain可以在集群中的任何节点上运行，当它收到消息时，Proto.Actor框架会自动激活它；当它一段时间没有收到消息时，框架会自动将它停用，释放资源。使用虚拟 Actor 模型，一个 Actor 被特别称为 "grain"。然而，grain 的实现与任何其他 Actor 的实现非常相似。一个显著的区别是，proto.actor 在初始消息接收时会自动启动 grain。

Grain 的这种特性使得它非常适合用于构建大规模的分布式系统。开发者只需要关注业务逻辑，而无需关心Grain的位置、生命周期和并发问题。

在`Proto.Actor`的集群中，Grain、Remote和Actor之间的关系如下：

- **Grain**：Grain是一种特殊的Actor，它的生命周期由框架自动管理。Grain可以在集群中的任何节点上运行，它的状态可以持久化，也可以在内存中保存。

- **Remote**：Remote是Proto.Actor的分布式组件，它允许Actor跨网络节点进行通信。Grain就是通过Remote在集群中的节点之间进行通信的。

- **Actor**：Actor是Proto.Actor的核心组件，它是一个可以处理消息的实体。Grain实际上就是一种特殊的Actor，它提供了更高级的抽象，使得开发者可以更加专注于业务逻辑。

在实际使用中，你可以根据需要选择使用Actor或Grain。如果你需要更高级的抽象和自动的生命周期管理，可以选择使用`Grain`；如果你需要更细粒度的控制，可以选择使用Actor。

### 1.6.2. EventStream

在Proto.Actor中，发布-订阅模式是通过EventStream组件实现的。EventStream是一个全局的消息通道，你可以在任何地方向它发布消息，也可以订阅它的消息。

以下是使用EventStream进行发布-订阅的基本步骤：

1. **订阅消息**：你可以通过EventStream.Subscribe函数订阅某种类型的消息。这个函数需要一个消息处理函数，当有消息发布时，这个函数会被调用。例如：

```go
system.EventStream.Subscribe(func(msg *MyMessage) {

  *// 处理消息*

})
```

2. **发布消息**：你可以通过EventStream.Publish函数发布消息。这个函数需要一个消息对象，这个消息会被发送给所有订阅了这种类型消息的处理函数。例如：

```go
system.EventStream.Publish(&MyMessage{Value: "Hello, World!"})
```

3. **取消订阅**：你可以通过EventStream.Unsubscribe函数取消订阅。这个函数需要一个之前通过Subscribe函数返回的订阅对象。例如：

```
subscription := system.EventStream.Subscribe(func(msg *MyMessage) {

  // 处理消息

})

system.EventStream.Unsubscribe(subscription)
```


以上就是`Proto.Actor`中发布-订阅模式的基本使用方法。在实际使用中，你可能需要处理更复杂的场景，如在`Actor`之间进行发布-订阅、处理订阅失败等。

### 1.6.3. Pub-Sub

`Proto.Actor`的集群 `Pub-Sub`（发布-订阅）模式是通过`EventStream`组件实现的。`EventStream`是一个全局的消息通道，你可以在任何地方向它发布消息，也可以订阅它的消息。



### 1.6.4. cluster provider

cluster 的核心是 `cluster provider`。

集群提供者（cluster provider）：通常是指一种软件或服务，负责管理和提供集群的基础设施。一个集群是由多个计算机节点组成的集合，这些节点协同工作以完成共同的任务。Cluster Provider 为创建、监控和维护这个集群提供了支持，使得节点能够了解集群中其他节点的状态、可用性以及成员身份等信息。

以下是一些 Cluster Provider 的功能和解释：

1. **集群管理：** Cluster Provider 负责创建、配置和管理整个集群。它可能提供用户界面或命令行工具，用于定义集群的规模、节点配置和其他相关属性。
2. **节点协调：** 集群中的节点需要协同工作以执行任务。Cluster Provider 负责确保节点之间的通信和协作，并在需要时自动进行节点的发现和注册。
3. **资源分配和负载均衡：** Cluster Provider 可能会处理集群中资源的分配和负载均衡，以确保任务在集群中均匀分布，并最大程度地利用集群中的计算资源。
4. **故障恢复：** 集群中的节点可能会因故障而失效，Cluster Provider 需要具备故障检测和自动恢复的机制，以保持集群的可靠性和稳定性。
5. **扩展性：** Cluster Provider 应该支持集群的动态扩展，允许根据需求添加或移除节点，以适应工作负载的变化。
6. **安全性：** 提供对集群中数据和通信的安全性管理，包括身份验证、授权和加密等方面的功能。
7. **监控和日志：** 提供集群性能和健康状态的监控功能，同时记录有关集群操作和事件的日志信息，以便进行故障排除和性能分析。

在这里，"cluster provider" 是负责维护和提供集群状态信息的组件。它允许应用程序中的节点通过与集群提供者进行交互，获取有关整个集群拓扑结构、节点状态的更新，并且使得集群中的节点能够协同工作。

**"topology"** 指的是拓扑结构，特指集群的结构或组织方式。在计算机科学和网络领域，拓扑结构描述了系统中各个节点之间的连接方式和关系。





#### 1.6.4.1. Kubernetes Provider

#### 1.6.4.2. Consul Provider

#### 1.6.4.3. AWS ECS Provider

#### 1.6.4.4. Seed Provider - Experimental



### 1.6.5. gossip

在分布式系统中，**gossip协议**是一种用于在网络中传播信息的分散式通信方法。它基于随机选择的节点之间的互相交流，以便将信息传播到整个网络。Gossip协议通常用于处理集群中的成员关系、状态同步、故障检测等问题。

在Proto Actor的Golang实现中，gossip协议用于实现分布式系统中的Actor之间的通信和协作。Proto Actor提供了一种称为`gossip.Gossip`的模块，其中包含了一些用于在Actor之间传播信息的功能。

在这个上下文中，"gossip" 是指在分布式系统中，Actor之间相互交换信息的一种机制。它允许不同的Actor之间共享状态信息，从而实现集群中的协作和协调。通过使用gossip协议，系统中的Actor可以更容易地了解其他Actor的状态，帮助实现一致性、容错性和可扩展性等特性。

理解gossip的关键概念包括：

1. **随机选择的节点交流信息**：Gossip协议中的节点（这里是指Actor）随机选择其他节点进行信息的交流。这种随机性确保了信息在整个网络中的分布。
2. **信息的传播**：通过节点之间的交流，信息被传播到整个网络。这种方式使得系统中的所有节点逐渐了解到所有其他节点的状态。
3. **自我修复性**：Gossip协议通常设计成具有自我修复性。如果某个节点长时间不可达，它的信息仍然可以通过其他节点传播，从而防止信息的丢失。

在Proto Actor中，gossip机制帮助实现了集群中的Actor之间的信息传递和状态同步，使得系统更具弹性，能够应对网络分区、故障和动态的集群成员变化等情况。通过这种方式，Proto Actor系统可以更好地适应复杂的分布式环境。

### 1.6.6. identity lookup

在Proto Actor框架中，"identity lookup" 是指根据Actor的**标识（identity）查找**特定Actor实例的过程。在Proto Actor中，每个Actor都有一个唯一的标识符，通常是一个字符串。通过这个标识符，你可以在系统中查找并与特定的Actor进行交互。

在Proto Actor的Golang实现中，有一个 `actor.PID` 结构表示一个Actor的标识符，其中 `PID` 是 "Process ID" 的缩写。`PID` 包含了用于定位Actor的信息，比如Actor的地址（Address）、ID等。你可以使用 `actor.PID` 来发送消息给特定的Actor实例。



### 1.6.7. autoMangment





-------------

Actor kind 是一种 actor 的类型，它定义了 actor 处理消息的方式。

> `Kind` 表示一个集群可以管理的 Actor 种类。

重启期间事件的详细顺序如下所示：

1. 挂起actor，在这之间将不能处理普通消息直到恢复过来
2. 调用实例的`PreRestart`钩子(默认发送终止请求给所有的孩子，同时调用 postStop)
3. 等待所有请求终结的孩子在`PreRestart`之间(使用`context.Stop()`真正的停止; 像所有children actor 终结一样, 上一次终结child 的提示 将会影响下一步的处理
4. 通过原来actor的创建函数创建一个新的actor。
5. 新实例调用 `PostRestart`，默认已经调用了`PreStart`。
6. 发送重启消息给所有在步骤三被杀死的孩子，重启孩子将会递归的遵循同一个 步骤2 的process 。
7. 恢复actor。



# 2. References

- offical website: https://proto.actor/docs/
- cluster: https://proto.actor/docs/cluster/
- offical golang package API: https://pkg.go.dev/github.com/asynkron/protoactor-go
- [Golang] protoactor-go 101:

  - How actors communicate with each other: https://blog.oklahome.net/2018/09/protoactor-go-messaging-protocol.html

  - How actor.Future works to synchronize concurrent task execution: https://blog.oklahome.net/2018/11/protoactor-go-how-future-works.html
  -  How proto.actor's clustering works to achieve higher availability: https://blog.oklahome.net/2021/05/protoactor-clustering.html
- How virtual actor frameworks deal with cluster topology change: https://www.etteplan.com/stories/how-virtual-actor-frameworks-deal-cluster-topology-change
- Microsoft Orleans: https://learn.microsoft.com/en-us/dotnet/orleans/overview
- 知乎深入解析actor 模型（一)： actor 介绍及在游戏行业应用：https://zhuanlan.zhihu.com/p/427806717
- 知乎深入解析actor 模型（二)： actor 在go 实践proto.Actor 源码解析：https://zhuanlan.zhihu.com/p/427817175
- Chat Example Using Proto.Actor: https://aneshas.medium.com/chat-example-using-proto-actor-5b42864c2d70
- 微软 Orleans 教程：https://orleans.azurewebsites.net/docs/index.html

