# SSH

SSH 由客户端和服务端的软件组成，客户端可以使用的软件有SecureCRT、putty、Xshell等，
服务器端运行的是一个 sshd 的服务，通过使用SSH，可以把所有传输的数据进行加密，而且也能够
防止 dns 和 IP 欺骗，SSH 传输的数据是经过压缩的，可以加快传输速度。

OpenSSH（即常说的ssh）常用配置文件有两个 `ssh_config` 和 `sshd_config`。其中 `/etc/ssh/ssh_config`为客户端配置文件，`/etc/ssh/sshd_config` 为服务器端配置文件。

```
Tips：
在 sshd_config 配置文件中，以 # 加空格开头的是注释信息，以 # 开头的是默认配置信息。
```

```apl
服务端配置文件如下：

[root@KF-CFT-AP2 ssh]# cat /etc/ssh/sshd_config
#	$OpenBSD: sshd_config,v 1.80 2008/07/02 02:24:18 djm Exp $

# This is the sshd server system-wide configuration file.  See
# sshd_config(5) for more information.

# This sshd was compiled with PATH=/usr/local/bin:/bin:/usr/bin

# The strategy used for options in the default sshd_config shipped with
# OpenSSH is to specify options with their default value where
# possible, but leave them commented.  Uncommented options change a
# default value.

################# SSH Server 的整体设定 ######################
#Port 22                    # 设置 sshd 监听的端口，出于安全考虑，端口指定为小于等于 65535，并且非 22     
#AddressFamily any
#ListenAddress 0.0.0.0      # 设设置 sshd 监听（绑定）的IP地址，0.0.0.0 表示监听所有IPv4的地址，出于安全考虑，设置为指定IP地址，而非所有地址
#ListenAddress ::           # IPV6 的地址

################## 说明主机的 Private Key 放置的档案 ###########
# Disable legacy (protocol version 1) support in the server for new
# installations. In future the default will change to require explicit
# activation of protocol 1
Protocol 2                          # 设置协议版本为SSH1或SSH2，SSH1 存在漏洞与缺陷，选择SSH2

############################ 私人密钥的文件 ###################
#HostKey for protocol version 1
#HostKey /etc/ssh/ssh_host_key
#HostKeys for protocol version 2
#HostKey /etc/ssh/ssh_host_rsa_key   # 服务器秘钥文件存放的路径， RSA 私钥
#HostKey /etc/ssh/ssh_host_dsa_key   # dsa 密钥


#Compression yes                     # 是否可以使用压缩指令
# Lifetime and size of ephemeral version 1 server key
#KeyRegenerationInterval 1h          # 多长时间后系统自动重新生成服务器的秘钥
#ServerKeyBits 1024                  # 定义服务器密钥的长度

# Logging
# obsoletes QuietMode and FascistLogging
#SyslogFacility AUTH 
SyslogFacility AUTHPRIV              # 当有人使用 ssh 登录系统时，ssh 会记录信息，记录类型为AUTHPRIV，sshd 服务日志存放在/var/log/secure
#LogLevel INFO                       # sshd 日志信息的级别

############################## 安全登录 ###########################
# Authentication:         # 限制用户必须在指定的时限内认证成功，0 表示无限制。默认值是 120 秒

#LoginGraceTime 2m        # 设置指定时间内没有成功登录，将会断开连接，默认单位为 秒
#PermitRootLogin yes      # 是否允许他人远程 ssh 登录 root 用户，默认是允许的
#StrictModes yes          # 设置ssh在接收登录请求之前是否检查用户根目录和rhosts文件的权限和所有权，                           
                          # 建议使用默认值"yes"来预防可能出现的低级错误。
#MaxAuthTries 6
#MaxSessions 10

#RSAAuthentication yes    # 是否开启RSA密钥验证，只针对SSH1
#PubkeyAuthentication yes # 是否开启公钥验证，如果使用公钥验证的方式登录时，则设置为yes
PubkeyAuthentication yes
#AuthorizedKeysFile	.ssh/authorized_keys  # 公钥验证文件的路径，与 PubkeyAuthentication 配合使用
#AuthorizedKeysCommand none
#AuthorizedKeysCommandRunAs nobody

############################# 安全验证 #######################
# For this to work you will also need host keys in /etc/ssh/ssh_known_hosts
#RhostsRSAAuthentication no   # 是否使用强可信主机认证(通过检查远程主机名和关联的用户名进行认证)。
                              # 仅用于SSH-1。这是通过在RSA认证成功后再检查 ~/.rhosts 或                                       
                              # /etc/hosts.equiv 进行认证的。出于安全考虑，建议使用默认值"no"
                              
# similar for protocol version 2
#HostbasedAuthentication no   # 与 RhostsRSAAuthentication 类似，但是仅可以用于SSH-2
# Change to yes if you don't trust ~/.ssh/known_hosts for
# RhostsRSAAuthentication and HostbasedAuthentication
#IgnoreUserKnownHosts no      # 设置ssh在进行RhostsRSAAuthentication安全验证时是否忽略用户                                  
                              # 的“/$HOME/.ssh/known_hosts”文件
# Don't read the user's ~/.rhosts and ~/.shosts files
#IgnoreRhosts yes             # 验证的时候是否使用“~/.rhosts”和“~/.shosts”文件

# To disable tunneled clear text passwords, change to no here!
#PasswordAuthentication yes   # 是否开启密码验证机制，如果用密码登录系统，则设置yes
PasswordAuthentication yes  
#PermitEmptyPasswords no      # 是否允许空密码登录系统，设置为 no，不允许

# Change to no to disable s/key passwords
#ChallengeResponseAuthentication yes
ChallengeResponseAuthentication no    #  是否允许质疑-应答(challenge-response)认证

########## 与 Kerberos 有关的参数设定，指定是否允许基于Kerberos的用户认证 ########
# Kerberos options
#KerberosAuthentication no
#KerberosOrLocalPasswd yes
#KerberosTicketCleanup yes
#KerberosGetAFSToken no
#KerberosUseKuserok yes

###### 与 GSSAPI 有关的参数设定，指定是否允许基于GSSAPI的用户认证，仅适用于SSH2 ####
# GSSAPI options
#GSSAPIAuthentication no         # 是否允许使用基于 GSSAPI 的用户认证，默认值为 no
#GSSAPICleanupCredentials yes
GSSAPICleanupCredentials yes
#GSSAPIStrictAcceptorCheck yes
#GSSAPIKeyExchange no

# Set this to 'yes' to enable PAM authentication, account processing, 
# and session processing. If this is enabled, PAM authentication will 
# be allowed through the ChallengeResponseAuthentication and
# PasswordAuthentication.  Depending on your PAM configuration,
# PAM authentication via ChallengeResponseAuthentication may bypass
# the setting of "PermitRootLogin without-password".
# If you just want the PAM account and session checks to run without
# PAM authentication, then enable this but set PasswordAuthentication
# and ChallengeResponseAuthentication to 'no'.
#UsePAM no
UsePAM yes                   # 是否启用 PAM 插件式认证模块


# Accept locale-related environment variables        # 接受环境变量，只有SSH-2协议支持环境变量的传递
AcceptEnv LANG LC_CTYPE LC_NUMERIC LC_TIME LC_COLLATE LC_MONETARY LC_MESSAGES
AcceptEnv LC_PAPER LC_NAME LC_ADDRESS LC_TELEPHONE LC_MEASUREMENT
AcceptEnv LC_IDENTIFICATION LC_ALL LANGUAGE
AcceptEnv XMODIFIERS

#AllowAgentForwarding yes    
#AllowTcpForwarding yes    # 是否允许允许tcp端口转发，默认为 yes，保护其他的tcp连接
#GatewayPorts no           # 是否允许远程客户端使用本地主机的端口转发功能，出于安全考虑，建议禁止

################### X-Window下的使用 ###############################
#X11Forwarding no           # 是否允许X11转发
#X11DisplayOffset 10        # 指定X11 转发的第一个可用的显示区(display)数字。默认值是 10
                            #  防止 sshd 占用了真实的 X11 服务器显示区，从而发生混淆。
#X11UseLocalhost yes

################# 登入后的设置 ####################################
#PrintMotd yes         # 打印登录提示信息，提示信息存储在 /etc/moed 文件中
#PrintLastLog yes      # 是否显示上次登录信息，默认为 yes
#TCPKeepAlive yes      # 是否持续连接，设置yes可以防止死连接
                       # 这种消息可以检测到死连接、连接不当关闭、客户端崩溃等异常。在这个情况下，任何                       
                       # 一端死掉后， SSH 可以立刻知道，而不会有僵尸程序的发生！
#UseLogin no           # 是否在交互式会话的登录过程中使用，默认值是"no"。
#UsePrivilegeSeparation yes  # 设置使用者的权限
#PermitUserEnvironment no
#Compression delayed     # 是否对通信进行加密，还是延迟到认证成功之后在加密，可用值：yes, delayed(默认), no
#ClientAliveInterval 0
#ClientAliveCountMax 3
#ShowPatchLevel no
#UseDNS yes                   # 是否禁止DNS反向解析，默认是 yes，一般会注释
#PidFile /var/run/sshd.pid    # 存放 sshd 守护进程的进程号文件，默认是：/var/run/sshd.pid
#MaxStartups 10               # 设置同时允许几个尚未登入的联机，当用户连上ssh但并未输入密码即为所谓                               
                              # 的联机，这个联机中，为了保护主机，所以需要设置最大值，预设为10个，
                              # 而已经建立联机的不计算入内。
#PermitTunnel no
#ChrootDirectory none

# no default banner path
#Banner none

# override default of no subsystems
#Subsystem	sftp	/usr/libexec/openssh/sftp-server   # 配置一个外部子系统，仅用于 SSH-2。例如：一个传输文件守护进程
Subsystem sftp internal-sftp

# Example of overriding settings on a per-user basis
#Match User anoncvs
#	X11Forwarding no
#	AllowTcpForwarding no
#	ForceCommand cvs server
```

CentOS7 默认安装的是 OpenSSH_7.4p1 版本SSH，而 CentOS6 默认安装的是 OpenSSH_5.3p1。



参考

- [sshd_config(5) — Linux manual page](https://man7.org/linux/man-pages/man5/sshd_config.5.html)
- [What is SSH Public Key authentication?](https://www.ssh.com/academy/ssh/public-key-authentication)  SSH.com 官方讲解 SSH 用法，很全面。
- [SSH远程登录配置文件sshd_config详解](https://blog.csdn.net/field_yang/article/details/51568861)
- [CentOS6.9下升级默认的OpenSSH操作记录（升级到OpenSSH_7.6p1）](https://cloud.tencent.com/developer/article/1193007)

