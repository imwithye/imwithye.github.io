---
title: "Deploy Self-hosted GitHub Actions Runner with ArgoCD and Akuity Platform"
date: 2024-12-25T15:00:00+08:00
tags: ["GitHub Actions", "Kubernetes", "ArgoCD", "Akuity"]
showToc: true
ShowReadingTime: true
ShowWordCount: true
---

**GitHub Actions** simplifies CI/CD but can become costly with GitHub-hosted runners. Self-hosted runners offer better cost control and customization.

This post covers deploying a self-hosted runner using **ArgoCD** and the **Akuity Platform**.

### Why ArgoCD and Akuity Platform?

ArgoCD ensures Kubernetes clusters align with Git-defined states, simplifying management and improving consistency. While ArgoCD is powerful, the Akuity Platform enhances it with added security, usability, and its **KubeVision** tool for efficient Kubernetes resource visualization and troubleshooting.

---

In the next section, we’ll walk through the step-by-step process of deploying a self-hosted GitHub Actions runner in Kubernetes using ArgoCD and the Akuity Platform. This approach provides a scalable, cost-effective, and easily manageable solution for running GitHub Actions workflows.

## 1. Prepare Your Kubernetes Cluster

To get started, ensure you have a Kubernetes cluster and the Akuity Agent installed. Additionally, verify that KubeVision is enabled for your Akuity instance and cluster. You can follow the [Akuity Platform ArgoCD guide](https://docs.akuity.io/getting-started/) and the [Akuity Platform KubeVision guide](https://docs.akuity.io/kubevision-getting-started/enable-kubevision-feature) to set up your cluster and enable KubeVision.

## 2. Create a GitHub Actions Runner Secret

To authenticate the GitHub Actions Runner with your GitHub organization, you need to create a GitHub App. Follow the steps outlined in the [GitHub Actions Runner Controller Authentication](https://docs.github.com/en/actions/hosting-your-own-runners/managing-self-hosted-runners-with-actions-runner-controller/authenticating-to-the-github-api) guide.

After creating the GitHub App, collect the following information:

- **App ID**
- **Installation ID**
- **Private Key**

Then, use these values to create a Kubernetes secret.

```bash
kubectl create namespace arc-runners
kubectl create secret generic pre-defined-secret \
   --namespace=arc-runners \
   --from-literal=github_app_id=123456 \
   --from-literal=github_app_installation_id=654321 \
   --from-literal=github_app_private_key='-----BEGIN RSA PRIVATE KEY-----********'
```

Note that both the App ID and Installation ID are numeric values, while the private key is the content of the downloaded .pem file.

## 3. Deploy the GitHub Actions Runner on ArgoCD

Now, open your ArgoCD UI and create a new application. Use the following manifest to deploy the GitHub Actions Runner. The app is sourced from [https://github.com/lumos-run/argo-apps](https://github.com/lumos-run/argo-apps).

Note that this application follows the **App of Apps** model. In this case:

- The destination must be set to `in-cluster` or `https://kubernetes.default.svc`.
- The namespace must be `argocd`.

You may customize the `destinationServer` to point to your own cluster, which is the cluster where the GitHub Actions Runner will be deployed.

```yaml
apiVersion: argoproj.io/v1alpha1
kind: Application
metadata:
  name: "arc-runner"
spec:
  destination:
    namespace: argocd
    server: https://kubernetes.default.svc
  source:
    path: arc-runner
    repoURL: https://github.com/lumos-run/argo-apps
    targetRevision: HEAD
    helm:
      values: "destinationServer: http://cluster-my:8001"
  project: default
  syncPolicy:
    automated:
      prune: true
      selfHeal: true
```
