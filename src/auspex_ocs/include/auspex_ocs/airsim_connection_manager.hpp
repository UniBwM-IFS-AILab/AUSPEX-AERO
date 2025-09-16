#ifndef AIRSIM_CONNECTION_MANAGER_HPP
#define AIRSIM_CONNECTION_MANAGER_HPP

#include <memory>
#include <mutex>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

/**
 * @brief Singleton connection manager for AirSim/Colosseum RPC client
 * 
 * This class manages a shared RPC connection to prevent conflicts when
 * multiple components need to communicate with the simulator simultaneously.
 * Uses reference counting to properly manage the connection lifecycle.
 */
class AirSimConnectionManager {
public:
    /**
     * @brief Get the shared RPC client instance
     * @return Shared pointer to the RPC client
     * @throws std::runtime_error if connection fails
     */
    static std::shared_ptr<msr::airlib::MultirotorRpcLibClient> getClient();
    
    /**
     * @brief Release a reference to the shared client
     * 
     * When the last reference is released, the connection is closed.
     */
    static void releaseClient();
    
    /**
     * @brief Check if the connection is active
     * @return true if connected, false otherwise
     */
    static bool isConnected();
    
    /**
     * @brief Force disconnect and cleanup
     * 
     * Use this for emergency cleanup or when you want to force
     * a reconnection on the next getClient() call.
     */
    static void forceDisconnect();

private:
    AirSimConnectionManager() = default;
    ~AirSimConnectionManager() = default;
    
    // Prevent copying and assignment
    AirSimConnectionManager(const AirSimConnectionManager&) = delete;
    AirSimConnectionManager& operator=(const AirSimConnectionManager&) = delete;
    
    static std::mutex mutex_;
    static std::shared_ptr<msr::airlib::MultirotorRpcLibClient> client_;
    static int reference_count_;
    static bool is_connected_;
    
    static void createConnection();
};

#endif // AIRSIM_CONNECTION_MANAGER_HPP
