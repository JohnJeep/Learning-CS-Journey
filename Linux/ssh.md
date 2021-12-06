# SSH

SSH 由客户端和服务端的软件组成，客户端可以使用的软件有SecureCRT、putty、Xshell等，
服务器端运行的是一个 sshd 的服务，通过使用SSH，可以把所有传输的数据进行加密，而且也能够
防止 dns 和 IP 欺骗，SSH 传输的数据是经过压缩的，可以加快传输速度。

其中服务器端的配置文件为 `/etc/ssh/sshd_config`

```sh
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
#Port 22                    # 设置sshd监听的端口      
#AddressFamily any
#ListenAddress 0.0.0.0      # 设置sshd服务器绑定的IP地址
#ListenAddress ::

################## 说明主机的 Private Key 放置的档案 ###########

# Disable legacy (protocol version 1) support in the server for new
# installations. In future the default will change to require explicit
# activation of protocol 1
Protocol 2      # 设置协议版本为SSH1或SSH2，SSH1存在漏洞与缺陷，选择SSH2

# HostKey for protocol version 1
#HostKey /etc/ssh/ssh_host_key
# HostKeys for protocol version 2
#HostKey /etc/ssh/ssh_host_rsa_key   # 服务器秘钥文件存放的路径， RSA 私钥
#HostKey /etc/ssh/ssh_host_dsa_key   # dsa 密钥

#Compression yes                     # 是否可以使用压缩指令

# Lifetime and size of ephemeral version 1 server key
#KeyRegenerationInterval 1h          # 多长时间后系统自动重新生成服务器的秘钥
#ServerKeyBits 1024                  # 定义服务器密钥的长度

# Logging
# obsoletes QuietMode and FascistLogging
#SyslogFacility AUTH 
SyslogFacility AUTHPRIV              # 记录来自sshd的消息的时候，是否给出“facility code”
#LogLevel INFO                       # sshd日志消息的级别

############################## 安全认证 ###########################
############################## 安全登录 ###########################
# Authentication:         # 限制用户必须在指定的时限内认证成功，0 表示无限制。默认值是 120 秒

#LoginGraceTime 2m        # 设定如果用户登录失败，在切断连接前服务器需要等待的时间，单位为 秒
#PermitRootLogin yes      # 能不能直接以超级用户ssh登录
#StrictModes yes          # 设置ssh在接收登录请求之前是否检查用户根目录和rhosts文件的权限和所有权，                           # 建议使用默认值"yes"来预防可能出现的低级错误。
#MaxAuthTries 6
#MaxSessions 10

#RSAAuthentication yes    # 是否开启RSA密钥验证，只针对SSH1
#PubkeyAuthentication yes # 是否开启公钥验证，如果使用公钥验证的方式登录时，则设置为yes
#AuthorizedKeysFile	.ssh/authorized_keys  # 公钥验证文件的路径，与PubkeyAuthentication配合使用
#AuthorizedKeysCommand none
#AuthorizedKeysCommandRunAs nobody

############################# 安全验证 #######################
# For this to work you will also need host keys in /etc/ssh/ssh_known_hosts
#RhostsRSAAuthentication no   # 是否使用强可信主机认证(通过检查远程主机名和关联的用户名进行认证)。
                              # 仅用于SSH-1。这是通过在RSA认证成功后再检查 ~/.rhosts 或                                       # /etc/hosts.equiv 进行认证的。出于安全考虑，建议使用默认值"no"
# similar for protocol version 2
#HostbasedAuthentication no   # 与 RhostsRSAAuthentication 类似，但是仅可以用于SSH-2
# Change to yes if you don't trust ~/.ssh/known_hosts for
# RhostsRSAAuthentication and HostbasedAuthentication
#IgnoreUserKnownHosts no      # 设置ssh在进行RhostsRSAAuthentication安全验证时是否忽略用户                                   # 的“/$HOME/.ssh/known_hosts”文件
# Don't read the user's ~/.rhosts and ~/.shosts files
#IgnoreRhosts yes             # 验证的时候是否使用“~/.rhosts”和“~/.shosts”文件

# To disable tunneled clear text passwords, change to no here!
#PasswordAuthentication yes   # 是否开启密码验证机制，如果用密码登录系统，则设置yes
#PermitEmptyPasswords no      # # 是否允许用口令为空的账号登录系统，设置no
PasswordAuthentication yes    

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

###### 与 GSSAPI 有关的参数设定，指定是否允许基于GSSAPI的用户认证，仅适用于SSH2####
# GSSAPI options
#GSSAPIAuthentication no
GSSAPIAuthentication yes
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
UsePAM yes      # 是否通过PAM验证

# 接受环境变量，只有SSH-2协议支持环境变量的传递
# Accept locale-related environment variables
AcceptEnv LANG LC_CTYPE LC_NUMERIC LC_TIME LC_COLLATE LC_MONETARY LC_MESSAGES
AcceptEnv LC_PAPER LC_NAME LC_ADDRESS LC_TELEPHONE LC_MEASUREMENT
AcceptEnv LC_IDENTIFICATION LC_ALL LANGUAGE
AcceptEnv XMODIFIERS

#AllowAgentForwarding yes    
#AllowTcpForwarding yes    # 是否允许允许tcp端口转发，保护其他的tcp连接
#GatewayPorts no           # 是否允许远程客户端使用本地主机的端口转发功能，出于安全考虑，建议禁止

################### X-Window下的使用 ###############################
#X11Forwarding no
X11Forwarding yes           # 是否允许X11转发
#X11DisplayOffset 10        # 指定X11 转发的第一个可用的显示区(display)数字。默认值是 10
                            #  防止 sshd 占用了真实的 X11 服务器显示区，从而发生混淆。
#X11UseLocalhost yes

################# 登入后的设置 ####################################
#PrintMotd yes         # 设置sshd是否在用户登录时显示“/etc/motd”中的信息，
                       # 可以选在在“/etc/motd”中加入警告的信息
#PrintLastLog yes      # 是否显示上次登录信息
#TCPKeepAlive yes      # 是否持续连接，设置yes可以防止死连接
                       # 这种消息可以检测到死连接、连接不当关闭、客户端崩溃等异常。在这个情况下，任何                        # 一端死掉后， SSH 可以立刻知道，而不会有僵尸程序的发生！
#UseLogin no           # 是否在交互式会话的登录过程中使用。默认值是"no"。
#UsePrivilegeSeparation yes  # 设置使用者的权限
#PermitUserEnvironment no
#Compression delayed
#ClientAliveInterval 0
#ClientAliveCountMax 3
#ShowPatchLevel no
#UseDNS yes                   # 是否使用dns反向解析
#PidFile /var/run/sshd.pid
#MaxStartups 10               # 设置同时允许几个尚未登入的联机，当用户连上ssh但并未输入密码即为所谓                               # 的联机，这个联机中，为了保护主机，所以需要设置最大值，预设为10个，
                              # 而已经建立联机的不计算入内。
#PermitTunnel no
#ChrootDirectory none

# no default banner path
#Banner none

# override default of no subsystems
#Subsystem	sftp	/usr/libexec/openssh/sftp-server

# Example of overriding settings on a per-user basis
#Match User anoncvs
#	X11Forwarding no
#	AllowTcpForwarding no
#	ForceCommand cvs server
Subsystem sftp internal-sftp

Match User L2Sftp

ChrootDirectory /root/L2BU/

ForceCommand internal-sftp

AllowTcpForwarding no

X11Forwarding no

PubkeyAuthentication yes
```



参考

[What is SSH Public Key authentication?](https://www.ssh.com/academy/ssh/public-key-authentication)  SSH.com 官方讲解 SSH 用法，很全面。

[SSH远程登录配置文件sshd_config详解](https://blog.csdn.net/field_yang/article/details/51568861)

