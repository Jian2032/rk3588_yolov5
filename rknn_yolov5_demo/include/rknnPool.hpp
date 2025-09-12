#ifndef RKNNPOOL_H
#define RKNNPOOL_H

#include "ThreadPool.hpp"
#include <vector>
#include <iostream>
#include <mutex>
#include <queue>
#include <memory>

// rknnModel模型类, inputType模型输入类型, outputType模型输出类型
template <typename rknnModel, typename inputType, typename outputType>
class rknnPool
{
private:
    // 线程池线程数
    int threadNum;
    // 模型文件
    std::string modelPath;
    // 任务编号，用于轮流分配模型实例
    long long id;
    // 互斥锁
    std::mutex idMtx, queueMtx;
    // 线程池
    std::unique_ptr<dpool::ThreadPool> pool;
    // 储存异步任务的future
    std::queue<std::future<outputType>> futs;
    // 存放多个模型的实例
    std::vector<std::shared_ptr<rknnModel>> models;

protected:
    // 获取下一个可用模型ID
    int getModelId();

public:
    rknnPool(const std::string modelPath, int threadNum);
    // 初始化模型和线程数
    int init();
    // 模型推理/Model inference
    int put(inputType inputData);
    // 获取推理结果/Get the results of your inference
    int get(outputType &outputData);
    ~rknnPool();
};

template <typename rknnModel, typename inputType, typename outputType>
rknnPool<rknnModel, inputType, outputType>::rknnPool(const std::string modelPath, int threadNum)
{
    this->modelPath = modelPath;
    this->threadNum = threadNum;
    this->id = 0;
}

template <typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::init()
{
    try
    {
        // 创建线程池
        this->pool = std::make_unique<dpool::ThreadPool>(this->threadNum);
        // 创建多个模型实例
        for (int i = 0; i < this->threadNum; i++)
            models.push_back(std::make_shared<rknnModel>(this->modelPath.c_str()));
    }
    catch (const std::bad_alloc &e)
    {
        std::cout << "Out of memory: " << e.what() << std::endl;
        return -1;
    }
    // 初始化模型/Initialize the model
    for (int i = 0, ret = 0; i < threadNum; i++)
    {
        ret = models[i]->init(models[0]->get_pctx(), i != 0);
        if (ret != 0)
            return ret;
    }

    return 0;
}

template <typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::getModelId()
{
    // 自动加锁解锁管理ID
    std::lock_guard<std::mutex> lock(idMtx);
    // 循环分配ID
    int modelId = id % threadNum;
    id++;
    return modelId;
}

template <typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::put(inputType inputData)
{
    std::lock_guard<std::mutex> lock(queueMtx);

    // 当队列长度大于10，保留最新帧删除旧帧
    if ((int)futs.size() >= 5) {
        futs.pop(); // 直接丢弃，不阻塞
    }
    // 提交异步任务
    futs.push(pool->submit(&rknnModel::infer, models[this->getModelId()], inputData));

    return 0;
}

template <typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::get(outputType &outputData)
{
    std::lock_guard<std::mutex> lock(queueMtx);
    if(futs.empty() == true)
        return 1;
    outputData = futs.front().get();
    futs.pop();
    return 0;
}

template <typename rknnModel, typename inputType, typename outputType>
rknnPool<rknnModel, inputType, outputType>::~rknnPool()
{
    while (!futs.empty())
    {
        outputType temp = futs.front().get();
        futs.pop();
    }
}

#endif
