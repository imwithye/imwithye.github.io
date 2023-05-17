---
title: "Asynchronous function in Unity"
date: 2020-11-16T11:30:03+00:00
tags: ["Unity", "Programming", "C#"]
showToc: true
ShowReadingTime: true
ShowWordCount: true
---

## Async and await keyword

Since Unity 2019, Unity introduces C# task and async/await keyword to MonoBehaviour. For Unity callback functions like `Start`, `Update`, now it supports the async version, and with the `async` keyword in the front, the function now will be dispatched asynchronously automatically by the engine.

```c#
private async void Start()
{
    Debug.Log("Start task delay 2 seconds");
    await Task.Delay(TimeSpan.FromSeconds(2));
    Debug.Log("Task delay 2 finished");
}
```

The function on the above will be executed and the first log shows immediately while the second log shows after 2 seconds.
The C# build-in HTTP library also provides a nice async wrapper, which can be used in the asynchronous MonoBehaviour as well.

```c#
private async void Start()
{
    var httpClient = new System.Net.Http.HttpClient();
    var resp = await httpClient.GetAsync("https://google.com");
    Debug.Log(resp.StatusCode);
}
```

With the magic `async` keyboard, we can replace the Unity default coroutine, which is implemented by a `yield` generator with a much cleaner and nicer programming style.

## Background task problem

However, the async function, similar to JavaScript, is not executed in another thread. Instead, the function executes on the main thread, and only when the await keyword appears, the function executes may or **MAY NOT** on the main thread.

```c#
private async void Start()
{
    Log("Start", "Task delay 2 seconds");
    await Task.Delay(TimeSpan.FromSeconds(2));
    Log("Start", "Task delay 2 finished");
    Log("Start", "Thread sleep 2 seconds");
    Thread.Sleep(TimeSpan.FromSeconds(2));
    Log("Start", "Thread sleep done");
}

private void Update()
{
    _frames++;
}

private void Log(string caller, string message)
{
    Debug.Log($"[Thread {Thread.CurrentThread.ManagedThreadId}][Frame {_frames}][Method {caller}] {message}");
}
```

Here we define a helper function Log to monitor the current thread and current frame count. The above function returns:

```
1. [Thread 1][Frame 0][Method Start] Task delay 2 seconds
2. [Thread 1][Frame 172][Method Start] Task delay 2 finished
3. [Thread 1][Frame 172][Method Start] Thread sleep 2 seconds
4. [Thread 1][Frame 172][Method Start] Thread sleep done
```

Notice the function does execute sequentially and asynchronously. Line 1 and line 2 shows the await keyword pushes the task to the background and resumes execution after 2 seconds as the frame count is different. However, line 3 and line 4 shows that the function runs still on the main thread, if there is a heavy computation task in the function, it blocks the whole system until it finishes.

## Workaround

Instead of lunching the task on the main thread, we can force it to lunch on a background thread and return a task to inform the main thread to await the result.

```c#
private async void Start()
{
    Log("Start", "Task delay 2 seconds");
    await Task.Delay(TimeSpan.FromSeconds(2));
    Log("Start", "Task delay 2 finished");
    Log("Start", "Thread sleep 2 seconds");
    await Task.Run(() => Thread.Sleep(TimeSpan.FromSeconds(2)));
    Log("Start", "Thread sleep done");
}
```

`Task.Run` function will lunch the lambda action on the background thread. Instead of blocking the execution of the main thread, the main thread Start function will pause here and await the `Task.Run` function to finish. Then it resumes afterward. The above function returns:

```
1. [Thread 1][Frame 0][Method Start] Task delay 2 seconds
2. [Thread 1][Frame 167][Method Start] Task delay 2 finished
3. [Thread 1][Frame 167][Method Start] Thread sleep 2 seconds
4. [Thread 1][Frame 378][Method Start] Thread sleep done
```

However, this is not perfect. Task.Run keyword requires the code wrapped into a function, which breaks the sequential code flow and makes the language more verbose. Secondly, it does now work with the Unity coroutine magic keywords, such as `WaitForEndOfFrame`.
**UniTask** solves these issues in an elegant and efficient way!

---

## UniTask

(UniTask)[https://github.com/Cysharp/UniTask] is written by Yoshifumi Kawai, a Japanese developer and Microsoft MVP for Visual C# since 2011. UniTask aims for providing an efficient allocation free async/await integration to Unity, and PlayerLoop based task(`UniTask.Yield`, `UniTask.Delay`, `UniTask.DelayFrame`, etc.) that enable to replace all coroutine operation.
UniTask provides a plug-and-play replacement for C# default Task library and Unity default coroutine. For example,

```c#
private IEnumerator<object> CoroutineSleep()
{
    Log("Coroutine", "Coroutin sleep 2 seconds");
    yield return new WaitForSeconds(2);
    Log("Coroutine", "Coroutin sleep done");
}
```

The code above can be easily converted into UniTask’s version

```c#
private async UniTask UniTaskSleep()
{
    Log("Coroutine", "UniTask sleep 2 seconds");
    await UniTask.Delay(TimeSpan.FromSeconds(2));
    Log("Coroutine", "UniTask sleep done");
}
```

UniTask provides PlayerLoop based tasks, for example, `await UniTask.WaitForEndOfFrame()`;, `await UniTask.NextFrame()`; so that you can apply them just like in the coroutine context.

## Background task in UniTask

Not only a nicer replacement for Unity coroutine, UniTask also provides a nicer background thread management. With special task SwitchToThreadPool, UniTask allows the thread changes current context to a thread pool thread and execute on the background.

```c#
private async void Start()
{
    Log("Start", "Task delay 2 seconds");
    await UniTask.Delay(TimeSpan.FromSeconds(2));
    Log("Start", "Task delay 2 finished");
    Log("Start", "Thread sleep 2 seconds");
    await UniTask.SwitchToThreadPool();
    Log("Start", "Going to sleep");
    Thread.Sleep(TimeSpan.FromSeconds(2));
    await UniTask.SwitchToMainThread();
    Log("Start", "Thread sleep done");
}
```

The result of above function

```
1. [Thread 1][Frame 0][Method Start] Task delay 2 seconds
2. [Thread 1][Frame 167][Method Start] Task delay 2 finished
3. [Thread 1][Frame 167][Method Start] Thread sleep 2 seconds
4. [Thread 68][Frame 168][Method Start] Going to sleep
5. [Thread 1][Frame 287][Method Start] Thread sleep done
```

UniTask will push the current execution context to the thread pool after `UniTask.SwitchToThreadPool()` and switch back to main thread after `UniTask.SwitchToMainThread()`.
UniTask also provides a handy tool, UniTask Tracker to monitor the task usage in the Unity Editor.
