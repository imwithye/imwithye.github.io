---
title: "K3s Cloud Master and Local Agent Setup"
date: 2024-09-30
tags: ["k8s", "k3s", "cluster"]
---

Setting up a cloud-based k3s master node with local agent nodes allows for easy management of distributed infrastructure. This setup is ideal for scenarios such as integrating a local GPU cluster or managing edge devices like vending machines, ensuring stable network connectivity without the need for additional software.

### Use Cases

1. **Local GPU Cluster**: You can add your local GPU cluster as a worker node to the cloud k3s master, enabling centralized scheduling of complex tasks without worrying about network issues.
   
2. **Edge Device Management**: If you have many edge devices, such as vending machines, you can connect them through local agent nodes to the cloud, allowing centralized management of the pods running on those devices, without having public IP address.

With built-in support for Flannel and WireGuard, k3s enables seamless networking without requiring additional software.

### Firewall Configuration

Before setting up your k3s cluster, it's crucial to configure your firewall to allow the necessary communication between nodes. Here are the key ports that need to be opened:

1. **UDP 51820**: This port is essential for WireGuard communication, which k3s uses for its networking backend.
2. **TCP 6443**: The Kubernetes API server port, used for cluster management.
3. **TCP 80 and 443**: These ports are used for HTTP and HTTPS traffic, allowing you to expose services from your cluster.

### Setting Up with k3s

First, install k3s on your cloud server and local agent:

```bash
# Set server external IP
export EXTERNAL_IP=$(curl -sSL ifconfig.me)
# Install on K3s master node
curl -sfL https://get.k3s.io | sh -s - --tls-san=$EXTERNAL_IP --node-external-ip=$EXTERNAL_IP --flannel-backend=wireguard-native
# Get join token
cat /var/lib/rancher/k3s/server/node-token


# Install on local agent node
# EXTERNAL_IP is the public IP of master node
curl -sfL https://get.k3s.io | K3S_URL=https://$EXTERNAL_IP:6443 K3S_TOKEN=<NODE_TOKEN> sh - agent
```

### Alternative: Setting Up with k3d

If you prefer a lightweight local development environment, k3d is an excellent option. It allows you to run k3s clusters within Docker containers, making it easy to create, manage, and test clusters directly on your local machine. However, it’s important to note that k3d can only function as a local agent node, not as a server node.

```bash
# Install on local agent node
# EXTERNAL_IP is the public IP of master node
K3D_FIX_DNS=0 k3d node create agent --cluster https://$EXTERNAL_IP:6443 --token <NODE_TOKEN>
```

### Conclusion

With k3s and k3d, you can easily set up and manage distributed Kubernetes clusters across cloud and edge environments, enabling seamless networking and centralized control for various deployment scenarios.
