<!--
 * @Author: JohnJeep
 * @Date: 2020-08-06 22:20:12
 * @LastEditTime: 2020-08-07 16:00:28
 * @LastEditors: Please set LastEditors
 * @Description: 设计模式学习
 * @FilePath: /DesignPattern.md
-->


## 单例模式
- 参考
  - [java单例模式](https://blog.csdn.net/czqqqqq/article/details/80451880)
  - [C++ 线程安全的单例模式总结](https://www.cnblogs.com/xiaolincoding/p/11437231.html)


- 定义：保证一个类只能生成一个唯一的实例对象，同时提供该实例访问的全局方法。
- 实现的步骤
  - 构造函数私有化，禁止他人创建
  - 提供一个全局的静态方法
  - 在类中定义一个静态指针，指向该类变量的静态变量指针
- 单例模式优点
  - 在内存中只有一个对象，节省内存空间；
  - 避免频繁的创建销毁对象，可以提高性能；
  - 避免对共享资源的多重占用，简化访问；
  - 为整个系统提供一个全局访问点。
- 注意事项
  - 在使用单例模式时，我们必须使用单例类提供的公有工厂方法得到单例对象，而不应该使用反射来创建，否则将会实例化一个新对象。此外，在多线程环境下使用单例模式时，应特别注意线程安全问题
  - 想要实现高效率的多线程安全的单例模式，应注意
    - 尽量减少同步块的作用域；
    - 尽量使用细粒度的锁


- 单线程下实现单例模式
  - 懒汉式：等到真正使用的时候才去创建实例，不用时不去主动创建
  - 饿汉式：在类加载初始化的时候就主动创建实例


- 多线程下实现单例模式
  - 多线程下，懒汉式的实现时不安全的，饿汉式方法实现是安全的。
  - 如何解决线程不安全的问题？
    - 使用 `synchronized方法`
      ```
      // 线程安全的懒汉式单例
      public class Singleton2 {
          private static Singleton2 singleton2;
          private Singleton2(){}
          // 使用 synchronized 修饰，临界资源的同步互斥访问
          public static synchronized Singleton2 getSingleton2(){
              if (singleton2 == null) {
                  singleton2 = new Singleton2();
              }
              return singleton2;
          }
      }

      // 优缺点
         1、运行效率低，因为同步块的作用域很大，锁的粒度有点粗
         2、保证了对临界资源的同步访问
      ```
    - 使用 `synchronized块`
      ```
      // 线程安全的懒汉式单例
      public class Singleton2 {
          private static Singleton2 singleton2;
          private Singleton2(){}
          public static Singleton2 getSingleton2(){
              synchronized(Singleton2.class){  // 使用 synchronized 块，临界资源的同步互斥访问
                  if (singleton2 == null) { 
                      singleton2 = new Singleton2();
                  }
              }
              return singleton2;
          }
      }

      // 行效率仍然比较低，事实上，和使用synchronized方法的版本相比，基本没有任何效率上的提高。
      ```
    - 使用 `内部类的懒汉式`
      ```
      // 线程安全的懒汉式单例
      public class Singleton5 {
          // 私有内部类，按需加载，用时加载，也就是延迟加载
          private static class Holder {
              private static Singleton5 singleton5 = new Singleton5();
          }
          private Singleton5() {
      
          }
          public static Singleton5 getSingleton5() {
              return Holder.singleton5;
          }

        // 效率比较高
      ```
    - 使用 `双重检测机制`
    ```
      // 线程安全的懒汉式单例
      public class Singleton3 {
      
          //使用volatile关键字防止重排序，因为 new Instance()是一个非原子操作，可能创建一个不完整的实例
          private static volatile Singleton3 singleton3;
      
          private Singleton3() {
          }
      
          public static Singleton3 getSingleton3() {
              // Double-Check idiom
              if (singleton3 == null) {
                  synchronized (Singleton3.class) {       // 1
                      // 只需在第一次创建实例时才同步
                      if (singleton3 == null) {       // 2
                          singleton3 = new Singleton3();      // 3
                      }
                  }
              }
              return singleton3;
          }
      }

      // 保证了单例，提高了效率
      // 必须使用volatile关键字修饰单例引用。目的：解决指令重排序的问题
    ``` 
  - 借助 ` ThreadLocal`
    ```
    // 线程安全的懒汉式单例
    public class Singleton4 {
    
      // ThreadLocal 线程局部变量
      private static ThreadLocal<Singleton4> threadLocal = new ThreadLocal<Singleton4>();
      private static Singleton4 singleton4 = null;

      private Singleton4(){}

      public static Singleton4 getSingleton4(){
          if (threadLocal.get() == null) {        // 第一次检查：该线程是否第一次访问
              createSingleton4();
          }
          return singleton4;
      }

      public static void createSingleton4(){
          synchronized (Singleton4.class) {
              if (singleton4 == null) {          // 第二次检查：该单例是否被创建
                  singleton4 = new Singleton4();   // 只执行一次
              }
          }
          threadLocal.set(singleton4);      // 将单例放入当前线程的局部变量中 
      }
    }

    // 作用：将临界资源线程局部化
    // 使用ThreadLocal的实现在效率上还不如双重检查锁定。
    ``` 

几种方式实现单例模式的对比
单例模式  | 是否线程安全 | 是否懒加载  | 是否防止反射构建
--- | --- | --- | ---
双重锁检测 | 是 | 是 | 否
静态内部类 | 是 | 是 | 否
枚举       | 是 | 否 | 是
