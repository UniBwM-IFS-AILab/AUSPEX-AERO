#include "auspex_ocs/airsim_connection_manager.hpp"
#include <thread>
#include <chrono>
#include <stdexcept>

// Static member definitions
std::mutex AirSimConnectionManager::mutex_;
std::shared_ptr<msr::airlib::MultirotorRpcLibClient> AirSimConnectionManager::client_;
int AirSimConnectionManager::reference_count_ = 0;
bool AirSimConnectionManager::is_connected_ = false;

std::shared_ptr<msr::airlib::MultirotorRpcLibClient> AirSimConnectionManager::getClient() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!client_ || !is_connected_) {
        createConnection();
    }
    
    reference_count_++;
    return client_;
}

void AirSimConnectionManager::releaseClient() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (reference_count_ > 0) {
        reference_count_--;
    }
    
    // Clean up when no more references exist
    if (reference_count_ <= 0 && client_) {
        try {
            client_.reset();
            is_connected_ = false;
        } catch (const std::exception&) {
            // Ignore cleanup errors
        }
        reference_count_ = 0;
    }
}

bool AirSimConnectionManager::isConnected() {
    std::lock_guard<std::mutex> lock(mutex_);
    return is_connected_ && client_ != nullptr;
}

void AirSimConnectionManager::forceDisconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (client_) {
        try {
            client_.reset();
        } catch (const std::exception&) {
            // Ignore cleanup errors
        }
    }
    
    is_connected_ = false;
    reference_count_ = 0;
}

void AirSimConnectionManager::createConnection() {
    // This method should only be called with mutex already locked
    
    try {
        // Create RPC client with optimized parameters for Colosseum multi-drone scenarios
        client_ = std::make_shared<msr::airlib::MultirotorRpcLibClient>("127.0.0.1", 41451, 15);
        
        // Initialize connection with retry mechanism
        int retry_count = 0;
        const int max_retries = 5;
        
        while (retry_count < max_retries) {
            try {
                client_->confirmConnection();
                is_connected_ = true;
                return;
            } catch (const std::exception& e) {
                retry_count++;
                if (retry_count >= max_retries) {
                    client_.reset();
                    is_connected_ = false;
                    throw std::runtime_error("Failed to connect to Colosseum after " + 
                                           std::to_string(max_retries) + " attempts: " + e.what());
                }
                
                // Progressive backoff: wait longer between retries
                std::this_thread::sleep_for(std::chrono::milliseconds(500 * retry_count));
            }
        }
    } catch (const std::exception& e) {
        client_.reset();
        is_connected_ = false;
        throw std::runtime_error("Failed to create AirSim connection: " + std::string(e.what()));
    }
}
