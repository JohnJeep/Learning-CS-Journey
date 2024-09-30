# 1. Linux 时钟

## 1.1. date

`date` 命令可以用来查看和手动设置日期、时间。命令格式如下：

```shel
# 查看日期
[root@zz-scf-iot-api02 log]# date
Thu Sep 26 10:32:11 CST 2024

# 修改时间
[root@node1 ~]# date -s "20240225 20:16:00"  #yyyymmdd hh:mm:ss
Tue Feb 25 20:16:00 CST 2024

```

**MM**：月份（两位数，01-12）

**DD**：日期（两位数，01-31）

**hh**：小时（两位数，00-23，24小时制）

**mm**：分钟（两位数，00-59）

**YYYY**：年份（四位数）

## 1.2. hwclock

Linux 系统的硬件时间。

```shell
# 查看硬件时间
root@DESKTOP-0S33AUT:/home/hacker# hwclock
2024-09-26 10:36:37.978831+08:00

# 以系统时间为基准，修改硬件时间。将新的系统时间写入硬件时钟（BIOS时钟）。这是为了确保系统在下次启动时使用正确的时间。
hwclock -w, --systohc   #  System Clock to Hardware Clock

# 以硬件时间为基准，修改系统时间
hwclock -s, --hctosys   # he Hardware Clock to System Clock

```





## 1.3. ntpd



## 1.4. 公共时间服务器

阿里云

阿里云时间服务器，授时信号来自GPS、北斗两套卫星信号，并配备原子钟守时，
以下7个域名提供服务，大家可以直接使用。

```
http://time1.aliyun.com
http://time2.aliyun.com
http://time3.aliyun.com
http://time4.aliyun.com
http://time5.aliyun.com
http://time6.aliyun.com
http://time7.aliyun.com

或者直接访问这个地址  time.pool.aliyun.com

// 实例
ntpdate -u time.pool.aliyun.com
```




# 2. References

- [ntp时间服务器 时间同步](https://www.cnblogs.com/centos2017/p/7896704.html)
- [Linux系统时间同步方法小结](https://www.cnblogs.com/williamjie/p/10768657.html)