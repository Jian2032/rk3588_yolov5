#ifndef RKNNPOOL_H
#define RKNNPOOL_H

#include "ThreadPool.hpp" // 引入线程池头文件
#include <vector>
#include <iostream>
#include <mutex>
#include <queue>
#include <memory>
#include <future>

// rknnModel: 模型类模板
// inputType: 模型输入类型（如 cv::Mat 或图像数据结构）
// outputType: 模型输出类型（如检测结果框、置信度等）
template <typename rknnModel, typename inputType, typename outputType>
class rknnPool
{
private:
    int threadNum;                                  // 使用的线程数量
    std::string modelPath;                          // 模型路径
    long long id;                                   // 推理编号计数器（用于选择模型实例）
    std::mutex idMtx, queueMtx;                     // id 和 futs 队列的互斥锁
    std::unique_ptr<dpool::ThreadPool> pool;        // 智能指针持有线程池实例
    std::queue<std::future<outputType>> futs;       // 存储异步任务的队列
    std::vector<std::shared_ptr<rknnModel>> models; // 多个模型实例（线程隔离）

protected:
    // 获取当前模型实例编号（用于轮询使用模型）
    int getModelId();

public:
    // 构造函数：传入模型路径和线程数量
    rknnPool(const std::string modelPath, int threadNum);

    // 初始化模型和线程池
    int init();

    // 推理任务入队（异步处理）
    int put(inputType inputData);

    // 获取推理结果（阻塞直到完成）
    int get(outputType &outputData);

    // 析构函数：等待并回收所有任务
    ~rknnPool();
};

// 构造函数：记录路径和线程数，初始化 id
template <typename rknnModel, typename inputType, typename outputType>
rknnPool<rknnModel, inputType, outputType>::rknnPool(const std::string modelPath, int threadNum)
{
    this->modelPath = modelPath;
    this->threadNum = threadNum;
    this->id = 0;
}

// 初始化线程池和多个模型实例
template <typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::init()
{
    try
    {
        // 创建线程池
        this->pool = std::make_unique<dpool::ThreadPool>(this->threadNum);
        // 初始化每个线程一个模型实例，防止线程间模型冲突
        for (int i = 0; i < this->threadNum; i++)
            models.push_back(std::make_shared<rknnModel>(this->modelPath.c_str()));
    }
    catch (const std::bad_alloc &e)
    {
        std::cout << "Out of memory: " << e.what() << std::endl;
        return -1;
    }

    // 模型初始化，第一个模型为主，其他模型共享上下文（pctx）
    for (int i = 0, ret = 0; i < threadNum; i++)
    {
        ret = models[i]->init(models[0]->get_pctx(), i != 0); // i != 0 表示是否共享
        if (ret != 0)
            return ret;
    }

    return 0;
}

// 获取下一个模型编号（轮询分配）
template <typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::getModelId()
{
    std::lock_guard<std::mutex> lock(idMtx);
    // 取模保证结果 modelId 总是在 [0, threadNum - 1] 之间循环。
    // 举例，如果 threadNum = 4，id 依次为：
    // 0 % 4 = 0
    // 1 % 4 = 1
    // 2 % 4 = 2
    // 3 % 4 = 3
    // 4 % 4 = 0 （回到第一个模型）
    // 5 % 4 = 1
    // 6 % 4 = 2
    int modelId = id % threadNum;
    id++;
    return modelId;
}

// 将推理任务提交到线程池并记录 future
template <typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::put(inputType inputData)
{
    std::lock_guard<std::mutex> lock(queueMtx);
    // 提交异步推理任务，使用选定的模型实例进行 infer
    futs.push(pool->submit(&rknnModel::infer, models[this->getModelId()], inputData));
    return 0;
}

// 获取最前面的推理结果（阻塞），并弹出已完成任务
template <typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::get(outputType &outputData)
{
    std::lock_guard<std::mutex> lock(queueMtx);
    if (futs.empty())
        return 1;
    outputData = futs.front().get(); // 阻塞等待推理完成
    futs.pop();                      // 移除完成的任务
    return 0;
}

// 析构函数：确保所有异步任务都已完成
template <typename rknnModel, typename inputType, typename outputType>
rknnPool<rknnModel, inputType, outputType>::~rknnPool()
{
    while (!futs.empty())
    {
        outputType temp = futs.front().get(); // 等待每个任务完成
        futs.pop();
    }
}

#endif
