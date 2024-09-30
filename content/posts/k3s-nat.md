---
title: "K3s Cloud Master and Local Agent Setup"
date: 2024-09-30T10:30:03+08:00
tags: ["k8s", "k3s", "cluster"]
---

Setting up a cloud-based k3s master node with local agent nodes allows for easy management of distributed infrastructure. This setup is ideal for scenarios such as integrating a local GPU cluster or managing edge devices like vending machines, ensuring stable network connectivity without the need for additional software.

### Use Cases

1. **Local GPU Cluster**: You can add your local GPU cluster as a worker node to the cloud k3s master, enabling centralized scheduling of complex tasks without worrying about network issues.
   
2. **Edge Device Management**: If you have many edge devices, such as vending machines, you can connect them through local agent nodes to the cloud, allowing centralized management of the pods running on those devices, without having public IP address.

With built-in support for Flannel and WireGuard, k3s enables seamless networking without requiring additional software.

### Setting Up with k3s

First, install k3s on your cloud server and local agent:

```bash
# Set server external IP
export EXTERNAL_IP=$(curl -sSL ifconfig.me)

# Install k3s master node
curl -sfL https://get.k3s.io | sh -s - --tls-san=$EXTERNAL_IP --node-external-ip=$EXTERNAL_IP --flannel-backend=wireguard-native

# Get join token
cat /var/lib/rancher/k3s/server/node-token

# Install on local agent node
curl -sfL https://get.k3s.io | K3S_URL=https://$EXTERNAL_IP:6443 K3S_TOKEN=<NODE_TOKEN> sh - agent
```

### Alternative: Setting Up with k3d

If you prefer a lightweight, local development environment, k3d is a great alternative. k3d allows you to run k3s clusters inside Docker containers, making it easy to create, manage, and test clusters on your local machine. This setup is perfect for scenarios where you need to simulate cloud environments or experiment with edge device management without needing dedicated cloud resources. Plus, k3d integrates seamlessly with k3s, so all networking and node management features, such as Flannel and WireGuard, are available without additional configuration.

```bash
# Set server external IP
export EXTERNAL_IP=$(curl -sSL ifconfig.me)

# Install k3d master node
k3d cluster create dev \
  --api-port 6443 \
  -p "443:443@loadbalancer" \
  -p "80:80@loadbalancer" \
  -p "51820:51820@loadbalancer" \
  --k3s-arg "--tls-san=$EXTERNAL_IP@server:*" \
  --k3s-arg "--node-external-ip=$EXTERNAL_IP@server:*" \
  --k3s-arg "--flannel-backend=wireguard-native@server:*"

# Get join token
docker exec k3d-dev-server-0 cat /var/lib/rancher/k3s/server/node-token

# Install on local agent node
K3D_FIX_DNS=0 k3d node create agent --cluster https://$EXTERNAL_IP:6443 --token <NODE_TOKEN>
```

### Conclusion

With k3s and k3d, you can easily set up and manage distributed Kubernetes clusters across cloud and edge environments, enabling seamless networking and centralized control for various deployment scenarios.
