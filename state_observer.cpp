#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>

#include <iostream>
#include <vector>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include <stdexcept>

enum class Priority {
    LOW = 0,
    MEDIUM = 1,
    HIGH = 2
};


class Monitor {
public:
    Monitor(const std::string& name, Priority priority)
        : name_(name), priority_(priority) {}
    virtual ~Monitor() = default;
    const std::string& getName() const { return name_; }

    virtual bool Update(int state) = 0;
    virtual void Handle(int state) = 0;
    virtual double get_a() const {return a_;}
    void set_a(double input) {
        a_ = input;
    }

    Priority getPriority() const { return priority_; }

protected:
    std::string name_{"name"};
    Priority priority_{0};
    int state_{0};
    double a_{0.0};
};

class OverValueMonitor : public Monitor {
public:
    OverValueMonitor(const std::string& name, Priority priority)
        : Monitor(name, priority) {}
    double get_a() const {return a_;}
    void set_a(double input) {
        a_ = input+1;
    }
    bool Update(int state) override {
        return state < 5;
    }
    void Handle(int state) override {
        std::cout << getName() << " (priority " << static_cast<int>(getPriority())
                  << ") detected anomaly: " << state << "\n";
    }

    double a_{0.0};

};

class StateObserver{
public:
    StateObserver(const std::string& name): name_(name) {}
    const std::string& getName() const { return name_; }
    virtual bool Update(int state) = 0;
    int getValue() const { return value_; }

protected:
    std::string name_{"name"};
    int value_{0};
};

class DataObserver : public StateObserver {
public:
    DataObserver(const std::string& name)
        : StateObserver(name) {}

protected:
    bool Update(int state) override {
        value_ = state + 1;
        return true;
    }
};

class JumpDetector{
public:
    JumpDetector(const std::string& name): name_(name) {}
    const std::string& getName() const { return name_; }
    virtual bool Update(int state) = 0;
    bool isJumped() const { return is_jumped; }

protected:
    std::string name_{"name"};
    bool is_jumped{0};
    int state_{0};
    int pre_state_{0};
};

class UpJumpDetector : public JumpDetector {
public:
    UpJumpDetector(const std::string& name)
        : JumpDetector(name) {}
    bool Update(int state) override {
        pre_state_ = state_;
        state_ = state;
        if(pre_state_!=state_ && state_ > pre_state_){
            is_jumped = true;
        }else{
            is_jumped = false;
        }
        return true;
    }
};

class Observer {
public:
    ~Observer() = default;

    void addMonitor(std::unique_ptr<Monitor> monitor) {
        const std::string& name = monitor->getName();
        if (monitors_index_.count(name)) {
            throw std::runtime_error("monitor name already exists: " + name);
        }

        Monitor* ptr = monitor.get();
        auto it = std::lower_bound(monitors_.begin(), monitors_.end(), monitor,
            [](const std::unique_ptr<Monitor>& a, const std::unique_ptr<Monitor>& b) {
                return a->getPriority() < b->getPriority();
            });
        monitors_.insert(it, std::move(monitor));
        monitors_index_[name] = ptr;
    }

    void addJumpDetector(std::unique_ptr<JumpDetector> jump_detector) {
        const std::string& name = jump_detector->getName();
        jump_dectctors_[name] = std::move(jump_detector);
    }

    void addStateObserver(std::unique_ptr<StateObserver> state_observer) {
        const std::string& name = state_observer->getName();
        state_observers_[name] = std::move(state_observer);
    }

    void updateAll(int state) {
        for (auto& [name, jump_dectctor] : jump_dectctors_) {
            jump_dectctor->Update(state);
        }
        for (auto& [name, state_observer] : state_observers_) {
            state_observer->Update(state);
        }
        for (auto& monitor : monitors_) {
            monitor->set_a(6.0);
            if(!monitor->Update(state)){
                monitor->Handle(state);
                break;
            }
        }
    }

    const Monitor& getMonitor(const std::string& name) const {
        auto it = monitors_index_.find(name);
        if (it == monitors_index_.end()) {
            throw std::runtime_error("Observer not found: " + name);
        }
        return *it->second;
    }
    const JumpDetector& getJumpDetector(const std::string& name) const {
        auto it = jump_dectctors_.find(name);
        if (it == jump_dectctors_.end()) {
            throw std::runtime_error("Jump Detector not found: " + name);
        }
        return *it->second;
    }
    const StateObserver& getStateObserver(const std::string& name) const {
        auto it = state_observers_.find(name);
        if (it == state_observers_.end()) {
            throw std::runtime_error("State Observer not found: " + name);
        }
        return *it->second;
    }

private:
    std::vector<std::unique_ptr<Monitor>> monitors_;
    std::unordered_map<std::string, Monitor*> monitors_index_;
    std::unordered_map<std::string, std::unique_ptr<JumpDetector>> jump_dectctors_;
    std::unordered_map<std::string, std::unique_ptr<StateObserver>> state_observers_;
};

int main() {
    Observer observer;

    observer.addMonitor(std::make_unique<OverValueMonitor>("over_value", Priority::HIGH));
    observer.addJumpDetector(std::make_unique<UpJumpDetector>("up_jump"));
    observer.addStateObserver(std::make_unique<DataObserver>("data"));

    std::vector<int> states = {1,1,1,2,2,2,5,1,1,1};
    for(int state:states){
        observer.updateAll(state);
        // std::cout << "state: " << state << " JumpDetector name: " << observer.getJumpDetector("up_jump").isJumped() << " StateObserver name: " << observer.getStateObserver("data").getValue()<<std::endl;
        std::cout << "a: " << observer.getMonitor("over_value").get_a() << std::endl;
    }

    return 0;
}