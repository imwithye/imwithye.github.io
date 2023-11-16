---
title: "基于TinyTex的中文本地Tex环境"
date: 2023-11-16
tags: ["LaTex", "随笔"]
---

在TinyTex之前，我已经换过无数个排版工具。从最开始的Word，到Pages，再到基于Markdown开发的MWeb和Ulysses。我很喜欢Ulysses，因为他既支持Markdown，又通过MathJax支持了LaTex语法。但是Markdown的排版能力还是非常有限的，尤其是`theorem`库的缺乏，Markdown只能作为一个快速笔记软件使用。

后来，我一直在使用Overleaf，不得不说Overleaf是一个非常不错的项目，至今市面上都没有更好的替代品。这也导致Overleaf的价格非常高，甚至学生价格都很高。作为一个研究生和普通的程序员的低频用户，肯定无法为这么高昂的价格付费。而单单免费版本，则又缺乏了类似GitHub同步之类的功能。之所以使用LaTex排版而不是Word，最大原因之一就是LaTex对Git优化，连Git都无法用了，Overleaf实在没有更多吸引我的地方。

之后，我同样使用了一些第三方的LaTex环境。比如在macOS上很好用的Texifier。Texifier这个项目不得不说也是花了很多心思，但可以看得出团队的预算不足，很多地方存在设计的瑕疵和开发的Bug。而且作为一款编辑器来说，他实在算不上功能强大。只不过在LaTex的开箱即用上面，至少做到了顺手。

我还使用了MacTex（或者是Tex Live）+ VScode + LaTex Workshop，从易用性到编辑器的熟悉程度以及到各种细节小问题的处理上，这一套方案基本是最佳方案，唯一的遗憾是庞大的MacTex和墙内的渣网速。在调研搜索过程中，看到了TinyTex这个发行版，TinyTex又有R语言的R Markdown背书，从文件大小到稳定性基本都满足了一个基本的Tex排版要求。

## 1. 下载TinyTex二进制发行版

TinyTex的发行版可以直接从GitHub, [https://github.com/rstudio/tinytex-releases](https://github.com/rstudio/tinytex-releases)下载获得。我下载的是`TinyTeX`版本，`TinyTeX-2`的体积过于庞大，`TinyTeX-1`和`TinyTeX-0`的预置package又太少。

下载完成后解压缩放到任意文件夹下，并且将`bin`目录添加到`PATH`变量中。我是用的是macOS，因此我的`PATH`为

```bash
export PATH=$PATH:$HOME/TinyTeX/bin/universal-darwin
```

## 2. 更新`tlmgr`源和安装中文支持

设置好环境变量后，重启终端就可以访问`tlmgr`。为了在国内更快的访问速度，可以修改`tlmgr`源为清华源，执行

```bash
tlmgr option repository https://mirrors.tuna.tsinghua.edu.cn/CTAN/systems/texlive/tlnet
```

然后安装`ctex`包获得中文支持

```bash
tlmgr install ctex
```

## 3. 安装VScod + LaTex Workshop

在VScode中安装LaTex Workshop插件，安装成功后打开任意`tex`文件就可以愉快的书写啦。

对于中文支持，可以使用`CJKutf8`

```latex
\documentclass{article}
\usepackage{CJKutf8}
\begin{document}
\begin{CJK*}{UTF8}{gbsn}

\section{前言}

\section{关于数学部分}
数学、中英文皆可以混排。You can intersperse math, Chinese and English (Latin script) without adding extra environments.

\end{CJK*}

\bigskip  %% Just some white space

You can also insert Latin text in your document

\bigskip  %% Just some white space

\begin{CJK*}{UTF8}{bsmi}
這是繁體中文。
\end{CJK*}
\end{document}
```

进行中英文混排。