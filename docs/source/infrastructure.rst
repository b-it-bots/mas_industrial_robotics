.. _infrastructure:

Infrastructure
##############


C025 Hub Server
===============

You can use our workstation in C025 lab if you need computing facilities such as 
GPU, but the available resouces are limited with 24 cores of CPU, 40GB RAM, and 2x
Nvidia 3090 with 24 GB GPU memory each. However, if you need more resources, please request 
access to `the university HPC cluster <https://wr0.wr.inf.h-brs.de/wr/index.html>`_.

We also provide a JupyterHub server on the workstation so that you do not have to 
setup the environment.

Hub Server
----------

The address of the JupyterHub server is `https://10.20.118.78:32025 <https://10.20.118.78:32025>`_ and 
LDAP is used for the login.

Multiple environments can be provided and is described in `here <https://github.com/b-it-bots/docker>`_. 
You can choose the one that meets your needs or PR to the aforementioned repository 
if you need to update the libraries. We can, of course, provide more environment 
options as per request.

`Deep Learning env with Tensorflow and Pytorch` is default environment and contains 
several deep learning libraries defined in `gpu-notebook <https://github.com/b-it-bots/docker/tree/master/gpu-notebook>`_.
The GPUs are accessible by all authorized users and there is no individual GPU resouce 
allocation.

Access
------

In order to get access to the hub, we need to add your LDAP username. 

.. hint::

  Admins can add new users by going to admin page `https://10.20.118.78:32025/hub/admin <https://10.20.118.78:32025/hub/admin>`_.

* Within the university network: 

  You can access the hub directly from the university network.

* Remote access
  
  * University VPN

    Follow `this instruction <https://ux-2s18.inf.h-brs.de/faq/vpn>`_ to setup VPN 
    in your local machine. Once you are connected to the university VPN, you can 
    open the Hub address.

  * SSH tunnel

    If you do not want to use VPN, you can port forward from your FB02 account to 
    C025 workstation

    .. code-block:: bash
      
      # replace account2s with your FBUID
      ssh -L 32025:10.20.118.78:32025 account2s@home.inf.h-brs.de

    Then, open browser and go to `https://localhost:32025 <https://localhost:32025>`_.

.. note::

  SSH access to the workstation is only possible with ssh key, so if you need it, 
  please send us your ssh key.

University Cluster
==================
