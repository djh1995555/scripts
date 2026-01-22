#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <string>
#include <memory>

// 基础接口
class IObserver {
public:
    virtual ~IObserver() = default;
    virtual void update() = 0;
};

class INamed {
public:
    virtual ~INamed() = default;
    virtual std::string getName() const = 0;
};

// Monitor类
class Monitor : public IObserver, public INamed {
public:
    Monitor(const std::string& name, int priority) : name(name), priority(priority) {}

    void update() override {
        std::cout << "Monitor " << name << " updated (priority: " << priority << ")\n";
        // 满足条件时采取行动的逻辑
    }

    std::string getName() const override { return name; }

    int getPriority() const { return priority; }

private:
    std::string name;
    int priority;
};

// StateObserver类
class StateObserver : public IObserver, public INamed {
public:
    StateObserver(const std::string& name) : name(name) {}

    void update() override {
        std::cout << "StateObserver " << name << " updated\n";
        // 观测状态的逻辑
    }

    std::string getName() const override { return name; }

private:
    std::string name;
};

// JumpDetector类
class JumpDetector : public IObserver, public INamed {
public:
    JumpDetector(const std::string& name) : name(name) {}

    void update() override {
        std::cout << "JumpDetector " << name << " updated\n";
        // 观测变量跳变的逻辑
    }

    std::string getName() const override { return name; }

private:
    std::string name;
};

// Observer管理类
class Observer {
public:
    void addMonitor(const std::string& name, int priority) {
        auto monitor = std::make_unique<Monitor>(name, priority);
        monitors.push_back(std::move(monitor));
        // 按优先级排序
        std::sort(monitors.begin(), monitors.end(),
                  [](const std::unique_ptr<Monitor>& a, const std::unique_ptr<Monitor>& b) {
                      return a->getPriority() < b->getPriority();
                  });
        nameToObserver[name] = monitors.back().get();
    }

    void addStateObserver(const std::string& name) {
        auto observer = std::make_unique<StateObserver>(name);
        stateObservers.push_back(std::move(observer));
        nameToObserver[name] = stateObservers.back().get();
    }

    void addJumpDetector(const std::string& name) {
        auto detector = std::make_unique<JumpDetector>(name);
        jumpDetectors.push_back(std::move(detector));
        nameToObserver[name] = jumpDetectors.back().get();
    }

    void updateAll() {
        // 按优先级更新Monitor
        for (const auto& monitor : monitors) {
            monitor->update();
        }
        // 更新StateObserver
        for (const auto& observer : stateObservers) {
            observer->update();
        }
        // 更新JumpDetector
        for (const auto& detector : jumpDetectors) {
            detector->update();
        }
    }

    IObserver* getObserver(const std::string& name) {
        auto it = nameToObserver.find(name);
        if (it != nameToObserver.end()) {
            return it->second;
        }
        return nullptr;
    }

private:
    std::vector<std::unique_ptr<Monitor>> monitors;
    std::vector<std::unique_ptr<StateObserver>> stateObservers;
    std::vector<std::unique_ptr<JumpDetector>> jumpDetectors;
    std::unordered_map<std::string, IObserver*> nameToObserver;
};

// 示例用法
int main() {
    Observer observer;

    // 注册Monitor，优先级为1和3（会自动排序）
    observer.addMonitor("Monitor1", 3);
    observer.addMonitor("Monitor2", 1);

    // 注册StateObserver和JumpDetector
    observer.addStateObserver("StateObserver1");
    observer.addJumpDetector("JumpDetector1");

    // 更新所有观察者
    observer.updateAll();

    // 通过名字访问对象
    auto obs = observer.getObserver("Monitor1");
    if (obs) {
        obs->update(); // 手动调用
    }

    return 0;
}