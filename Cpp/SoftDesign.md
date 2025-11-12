<!--
 * @Author: JohnJeep
 * @Date: 2021-08-22 00:01:17
 * @LastEditTime: 2025-04-04 19:24:12
 * @LastEditors: JohnJeep
 * @Description: C++ 软件设计
-->

# 1. C++ 软件设计


## 1.1. 简单就是最好的 ---> 大道至简。
API 是软件组件的接口，隐藏了实现这个接口所需的内部细节。

API 必须拥有良好的设计、文档、回归测试、并且保证发布之间的稳定性。


## 1.2. 重用哲学
设计自己和其它程序员可以重复使用的代码。重用设计的准则：
- 编写一次，经常使用。
- 尽量避免代码重复。
- 不要重复写在自己写过的代码（Don't repeat yourself）。


为什么要重用设计代码？
- 重用设计可以节约金钱和时间。
- 缺乏重用性会导致代码重复。


## 1.3. 如何设计可重用代码
对于设计可重用代码而言，最重要的策略是**抽象**。设计代码时，需要考虑将    接口和实现进行分离，使代码更容易使用，程序员使用时不需要理解其内部实现细节。

​ 抽象将代码分为**接口**和**实现**，因此设计可重用代码会关注这两个领域。代码实现时思考：如何做到恰到好处的设计代码结构？考虑使用什么样的类层次结构？需要使用模板吗？如何将代码切分割为子系统？设计接口时思考：设计分接口是**库**还是**代码**的“入口“，程序员使用这个接口时，应该给提供什么样的功能。


## 1.4. 软件设计考虑的原则
- 可扩展性（scalability）：代码之间耦合低。
- 可移植性（portability）：不同平台上运行。
- 安全性：产品发布后，有没有做相关保护的措施。例如，当前发布的产品有问题，如何快速回滚到原来的版本，最大限度减少产品发布的风险。
- 可维护性：方便组内其它的成员和后续维护代码的人员的快速理解和熟悉你的代码。
- 可复用性


## 1.5. 优美命名
- launch
- wrapper
- crontab：定时器
- schedule
- task
- helper
- register
  - register_keybord_callback
- callback
- start
- cancel
- detail
- grace-period：宽限期
- spin


### 1.5.1. 管理类命名
- starter、bootstrap

  一般作为程序启动器使用，或者作为启动器的基类。通俗来说，可以认为是main函数的入口。

  ```
  AbstractBootstrap
  ServerBootstrap
  MacosXApplicationStarter
  DNSTaskStarter
  ```

- processor

  某一类功能的处理器，用来表示某个处理过程，是一系列代码片段的集合。

  ```
  CompoundProcessor
  BinaryComparisonProcessor
  DefaultDefaultValueProcessor
  ```

- manager

  对有生命状态的对象进行管理，通常作为某一类资源的管理入口。

  ```
  AccountManager
  DevicePolicyManager
  TransactionManager
  ```

- holder

  表示持有某个或者某类对象的引用，并可以对其进行统一管理。

  ```
  QueryHolder
  InstructionHolder
  ViewHolder
  ```

- provider

  Provider = Strategy + Factory Method。它更高级一些，把策略模式和方法工厂揉在了一块，让人用起来很顺手。Provider一般是接口或者抽象类，以便能够完成子实现。

  ```
  AccountFeatureProvider
  ApplicationFeatureProviderImpl
  CollatorProvider
  ```

- registrar

  注册并管理一系列资源。

  ```
  ImportServiceRegistrar
  IKryoRegistrar
  PipelineOptionsRegistrar
  ```

- Engine

  一般是核心模块，用来处理一类功能。引擎是个非常高级的名词，一般的类是没有资格用它的。

  ```
  ScriptEngine
  DataQLScriptEngine
  C2DEngine
  ```

- Task

  某个任务，通常是个runnable

  ```
  WorkflowTask
  FutureTask
  ForkJoinTask
  ```


### 1.5.2. 传播类命名
- context

  如果你的程序执行，有一些变量，需要从函数执行的入口开始，一直传到大量子函数执行完毕之后。这些变量或者集合，如果以参数的形式传递，将会让代码变得冗长无比。这个时候，你就可以把变量统一塞到Context里面，以单个对象的形式进行传递。

  ```
  AppContext
  ServletContext
  ApplicationContext
  ```

- propagator

  传播，繁殖。用来将context中传递的值进行复制，添加，清除，重置，检索，恢复等动作。通常，它会提供一个叫做propagate的方法，实现真正的变量管理。

  ```
  TextMapPropagator
  FilePropagator
  TransactionPropagator
  ```


### 1.5.3. 回调类命名
- callback：用于响应某类消息，进行后续处理。

- handler：表示持有真正消息处理逻辑的对象，它是有状态的。

- trigger：代表某类事件的处理，属于 handler。通常不会出现在类的命名中。

- listener：通常在观察者模式中用来表示特定的含义

  ```
  ChannelHandler
  SuccessCallback
  CronTrigger
  EventListener
  ```

- aware

  Aware就是感知的意思，一般以该单词结尾的类，都实现了Aware接口。

  ```
  ApplicationContextAware
  ApplicationStartupAware
  ApplicationEventPublisherAware
  ```


### 1.5.4. 监控类命名
- metric

  表示监控数据，比用 Monitor 要优雅点。

  ```
  TimelineMetric
  histogramMetric
  ```

- estimator

  估计 、统计。用于计算某一类统计数值的计算器。

  ```
  ConditionalDensityEstimator
  FixedFrameRateEstimator
  NestableLoadProfileEstimator
  ```

- accumulator

  用来缓存累加的中间计算结果，并提供读取通道。

  ```
  AbstractAccumulator
  StatsAccumulator
  TopFrequencyAccumulator
  ```

- tracker

  一般用于日志记录的追踪。

  ```
  VelocityTracker
  RocketTracker
  MediaTracker
  ```


### 1.5.5. 内存管理类命名
- allocator

  分配器，表示内存的分配。

  ```
  AbstractByteBufAllocator
  ArrayAllocator
  RecyclingIntBlockAllocator
  ```

- chunk

  表示一块内存。

  ```
  EncryptedChunk
  ChunkFactory
  MultiChunk
  ```

- arena

  英文是舞台、竞技场的意思。由于Linux把它用在内存管理上发扬光大，它普遍用于各种存储资源的申请、释放与管理。为不同规格的存储chunk提供舞台，好像也是非常形象的表示。

  关键是，这个词很美，作为后缀让类名显得很漂亮。

  ```
  BookingArena
  StandaloneArena
  PoolArena
  ```

- pool

  池子，用于内存池、连接池、线程池等。

  ```
  ConnectionPool
  ObjectPool
  MemoryPool
  ```


### 1.5.6. 过滤检测类命名
程序收到的事件和信息是非常多的，有些是合法的，有些需要过滤扔掉。根据不同的使用范围和功能性差别，过滤操作也有多种形式。你会在框架类代码中发现大量这样的名词。

- pipeline, chain

  ```
  Pipeline
  ChildPipeline
  DefaultResourceTransformerChain
  FilterChain
  ```

- filter

  过滤器，用来筛选某些满足条件的数据集，或者在满足某些条件的时候执行一部分逻辑。

  ```
  FilenameFilter
  AfterFirstEventTimeFilter
  ScanFilter
  ```

- interceptor

  拦截器。

  ```
  HttpRequestInterceptor
  ```

- evaluator

  评估器。可用于判断某些条件是否成立，一般内部方法 `evaluate` 会返回 `bool` 类型。比如你传递进去一个非常复杂的对象，或者字符串，进行正确与否的判断。

  ```
  ScriptEvaluator
  SubtractionExpressionEvaluator
  StreamEvaluator
  ```

- detector

  探测器。用来管理一系列探测性事件，并在发生的时候能够进行捕获和响应。比如Android的手势检测，温度检测等

  ```
  FileHandlerReloadingDetector
  TransformGestureDetector 
  ScaleGestureDetector
  ```


### 1.5.7. 结构类命名
- cache

  大块的缓存。常见的缓存算法有LRU、LFU、FIFO等。

  ```
  LoadingCache
  EhCacheCache
  ```

- buffer

  buffer是缓冲，不同于缓存，它一般用在数据写入阶段。

  ```
  ByteBuffer
  RingBuffer
  DirectByteBuffer
  ```

- composite

  将相似的组件进行组合，并以相同的接口或者功能进行暴露。

  ```
  CompositeData
  CompositeMap
  ScrolledComposite
  ```

- wrapper

  用来包装某个对象，做一些额外的处理，以便增加或者去掉某些功能。

  ```
  IsoBufferWrapper
  ResponseWrapper
  MavenWrapperDownloader 
  ```

- tuple

  元组。

  ```
  Tuple2
  Tuple3
  ```

- aggregator

  聚合器，做一些聚合计算。如分库分表中的sum，max，min等聚合函数的汇集。

  ```
  BigDecimalMaxAggregator
  PipelineAggregator
  TotalAggregator
  ```

- iterator

  迭代器。在数据集很大的时候，需要进行深度遍历，迭代器可以说是必备的。使用迭代器还可以在迭代过程中安全的删除某些元素。

  ```
  BreakIterator
  StringCharacterIterator
  ```

- batch

  批量执行请求或对象。

  ```
  SavedObjectBatch
  BatchRequest
  ```

- limiter

  限流器，使用漏桶算法或者令牌桶来完成平滑的限流。

  ```
  DefaultTimepointLimiter
  RateLimiter
  TimeBasedLimiter
  ```


### 1.5.8. 解析类命名
- converter, resolver

  转换和解析。一般用于不同对象之间的格式转换，把一类对象转换成另一类。注意它们语义上的区别，一般特别复杂的转换或者有加载过程的需求，可以使用Resolver。

  ```
  DataSetToListConverter
  LayoutCommandLineConverter
  InitRefResolver
  MustacheViewResolver
  ```

- parser

  用于非常复杂的解析器。比如解析 DSL。

  ```
  SQLParser
  JSONParser
  ```

- customizer

  表示对某个对象进行特别的配置。由于这些配置过程特别的复杂，值得单独提取出来进行自定义设置。

  ```
  ContextCustomizer
  DeviceFieldCustomizer
  ```

- formatter

  格式化类。主要用于字符串、数字或者日期的格式化处理工作。

  ```
  DateFormatter
  StringFormatter
  ```


### 1.5.9. 网络命名
- packet

  用于网络编程中的数据包。

  ```
  DhcpPacket
  PacketBuffer
  ```

- protocol

  表示某个协议。

  ```
  RedisProtocol
  HttpProtocol
  ```

- Encoder、Decoder、Codec

  编码解码器。

  ```
  RedisEncoder
  RedisDecoder
  RedisCodec
  ```

- Request，Response

  网络的请求与响应。


### 1.5.10. 其它
- util：表示工具类，一般是无状态的。
- helper：创建实例时才使用
  ```
  HttpUtil
  TestKeyFieldHelper
  CreationHelper
  ```

- invoker, invocation

  invoker是一类接口，通常会以反射或者触发的方式，执行一些具体的业务逻辑。通过抽象出invoke方法，可以在invoke执行之前对入参进行记录或者处理；在invoke执行之后对结果和异常进行处理，是AOP中常见的操作方式。

  ```
  MethodInvoker
  Invoker
  ConstructorInvocation
  ```

- selector

  根据一系列条件，获得相应的同类资源。

  ```
  X509CertSelector
  NodeSelector
  ```

- reporter

  用来汇报某些执行结果。

  ```
  ExtentHtmlReporter
  MetricReporter
  ```

- accessor

  封装了一系列get和set方法的类。但Accessor类一般是要通过计算来完成get和set，而不是直接操作变量。这适合比较复杂的对象存取服务。

  ```
  ComponentAccessor
  StompHeaderAccessor
  ```

- generator

  生成器。一般用于生成代码，生成id等。

  ```
  CodeGenerator
  CipherKeyGenerator
  ```


## 1.6. 实践经验指导
- 代码一定要做单元测试，测试代码尽量覆盖所有的分支（覆盖率尽量高），可以采用 GoogleTest 单元测试框架。
- 代码最好用 valgrind 等工具跑一遍，检查代码有没有内存泄漏风险和异常。
- 尽量使用现代 C++（C++11 以上）进行C++编程，开发效率，性能，安全性都有极大提高。
- 尽量使用智能指针，用 RAII 模式管理对象生命周期，静态 GC（智能指针，RAII，move语义）。
- 理解C++对象的内存模型和布局，方便定位和解决各种C++内存问题。
- 异常是一个即安全又危险的特性，请谨慎使用。
- 熟悉常见的设计模式和C++特有的设计范式（C++Idioms），帮助自己设计构建更好的系统，对代码进行必要的重构。


# 2. References
- 微信公众号 优雅的代码命名：https://mp.weixin.qq.com/s/-Se0olV03HLGy-Y_-60lOw