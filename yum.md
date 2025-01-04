```shell
wget https://vault.centos.org/6.6/os/x86_64/Packages/python-urlgrabber-3.9.1-9.el6.noarch.rpm

wget https://vault.centos.org/6.6/os/x86_64/Packages/python-iniparse-0.3.1-2.1.el6.noarch.rpm

wget https://vault.centos.org/6.6/os/x86_64/Packages/yum-plugin-fastestmirror-1.1.30-30.el6.noarch.rpm

wget https://vault.centos.org/6.6/os/x86_64/Packages/yum-metadata-parser-1.1.2-16.el6.x86_64.rpm

wget https://vault.centos.org/6.6/os/x86_64/Packages/yum-3.2.29-60.el6.centos.noarch.rpm

rpm -ivh python-iniparse-0.3.1-2.1.el6.noarch.rpm

rpm -ivh python-urlgrabber-3.9.1-9.el6.noarch.rpm

rpm -ivh yum-3.2.29-60.el6.centos.noarch.rpm yum-metadata-parser-1.1.2-16.el6.x86_64.rpm yum-plugin-fastestmirror-1.1.30-30.el6.noarch.rpm


wget https://vault.centos.org/6.6/os/x86_64/Packages/python-libs-2.6.6-52.el6.x86_64.rpm

wget https://vault.centos.org/6.6/os/x86_64/Packages/python-2.6.6-52.el6.x86_64.rpm

wget https://vault.centos.org/6.6/os/x86_64/Packages/python-devel-2.6.6-52.el6.x86_64.rpm

rpm -ivh yum-3.2.29-60.el6.centos.noarch.rpm yum-metadata-parser-1.1.2-16.el6.x86_64.rpm yum-plugin-fastestmirror-1.1.30-30.el6.noarch.rpm python-urlgrabber-3.9.1-9.el6.noarch.rpm python-iniparse-0.3.1-2.1.el6.noarch.rpm python-libs-2.6.6-52.el6.x86_64.rpm python-2.6.6-52.el6.x86_64.rpm python-devel-2.6.6-52.el6.x86_64.rpm --force --nodeps
```
