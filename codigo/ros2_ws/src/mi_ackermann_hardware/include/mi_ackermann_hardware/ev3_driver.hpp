#ifndef EV3_DRIVER_HPP
#define EV3_DRIVER_HPP

#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <stdexcept>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Constantes de conversión
// ─────────────────────────────────────────────────────────────────────────────

constexpr double COUNTS_PER_REV        = 360.0;
constexpr double LARGE_MOTOR_MAX_RAD_S = 17.80;
constexpr double MEDIUM_MOTOR_MAX_RAD_S = 26.18;

// ─────────────────────────────────────────────────────────────────────────────
// Tipo de motor — necesario para usar la constante correcta
// ─────────────────────────────────────────────────────────────────────────────
enum class EV3MotorType {
    LARGE,   // motor de tracción — ruedas traseras
    MEDIUM   // motor de dirección — servo delantero
};

// ─────────────────────────────────────────────────────────────────────────────
// Protocolo TCP (texto plano, una línea por ciclo)
//
//   RPi5 → EV3:  "vel_traccion_pct,pos_direccion_deg\n"
//                 ejemplo: "45.2,15.0\n"
//
//   EV3 → RPi5:  "pos_traccion_counts,vel_traccion_pct,pos_direccion_deg\n"
//                 ejemplo: "1523,44.8,14.9\n"
// ─────────────────────────────────────────────────────────────────────────────

class EV3Driver {
public:

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // Recibe IP del EV3 y puerto del servidor TCP que corre en ev3dev
    // ─────────────────────────────────────────────────────────────────────────
    EV3Driver(const std::string & ip, int port, EV3MotorType traction_type = EV3MotorType::LARGE)
    : ip_(ip), port_(port), sock_fd_(-1), traction_type_(traction_type)
    {
        // inicializa los valores internos en cero
        traction_position_rad_ = 0.0;
        traction_velocity_rad_s_ = 0.0;
        steering_position_rad_ = 0.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // init() — abre la conexión TCP con el EV3
    // Equivalente a openPort() + setBaudRate() en el XL330Driver
    // Retorna 0 si todo fue bien, -1 si falló
    // ─────────────────────────────────────────────────────────────────────────
    int init()
    {
        std::cout << "[EV3Driver] Conectando con EV3 en " << ip_ << ":" << port_ << std::endl;

        sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_fd_ < 0) {
            std::cerr << "[EV3Driver] Error al crear socket: " << strerror(errno) << std::endl;
            return -1;
        }

        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family      = AF_INET;
        server_addr.sin_port        = htons(port_);

        if (inet_pton(AF_INET, ip_.c_str(), &server_addr.sin_addr) <= 0) {
            std::cerr << "[EV3Driver] IP inválida: " << ip_ << std::endl;
            return -1;
        }

        if (connect(sock_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "[EV3Driver] No se pudo conectar al EV3: " << strerror(errno) << std::endl;
            sock_fd_ = -1;
            return -1;
        }

        std::cout << "[EV3Driver] Conexión establecida con el EV3." << std::endl;
        return 0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // close() — cierra la conexión TCP
    // Equivalente a deactivate() en el XL330Driver
    // ─────────────────────────────────────────────────────────────────────────
    void close()
    {
        if (sock_fd_ >= 0) {
            ::close(sock_fd_);
            sock_fd_ = -1;
            std::cout << "[EV3Driver] Conexión cerrada." << std::endl;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // setTractionVelocity() — envía velocidad de tracción al EV3
    // Equivalente a setTargetVelocityRadianPerSec() en el XL330Driver
    //
    // Parámetro: velocidad en rad/s (positivo = adelante, negativo = atrás)
    // ─────────────────────────────────────────────────────────────────────────
    void setTractionVelocity(double velocity_rad_s)
    {
        traction_cmd_rad_s_ = velocity_rad_s;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // setSteeringPosition() — envía ángulo de dirección al EV3
    // Equivalente a setTargetPositionRadian() en el XL330Driver
    //
    // Parámetro: ángulo en radianes (0 = recto, positivo = izquierda)
    // ─────────────────────────────────────────────────────────────────────────
    void setSteeringPosition(double position_rad)
    {
        steering_cmd_rad_ = position_rad;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // sendAndReceive() — envía comandos y recibe estados en un solo ciclo
    // Este método va dentro de write() y read() del Hardware Component
    //
    // Retorna true si la comunicación fue exitosa
    // ─────────────────────────────────────────────────────────────────────────
    bool sendAndReceive()
    {
        if (sock_fd_ < 0) {
            std::cerr << "[EV3Driver] Sin conexión." << std::endl;
            return false;
        }

        // ── 1. Convierte comandos a las unidades que entiende el EV3 ──────────

        // tracción: rad/s → porcentaje [-100, 100]
        double vel_pct = (traction_type_ == EV3MotorType::LARGE)
                         ? (-traction_cmd_rad_s_ / LARGE_MOTOR_MAX_RAD_S * 100.0)
                         : (traction_cmd_rad_s_ / MEDIUM_MOTOR_MAX_RAD_S * 100.0);

        // dirección: rad → grados
        double dir_deg = steering_cmd_rad_ * (180.0 / M_PI);

        // ── 2. Arma y envía el mensaje ────────────────────────────────────────
        // formato: "vel_pct,dir_deg\n"
        std::string msg = std::to_string(vel_pct) + "," + std::to_string(dir_deg) + "\n";

        ssize_t sent = send(sock_fd_, msg.c_str(), msg.size(), 0);
        if (sent < 0) {
            std::cerr << "[EV3Driver] Error al enviar: " << strerror(errno) << std::endl;
            return false;
        }

        // ── 3. Recibe la respuesta del EV3 ────────────────────────────────────
        // formato: "pos_counts,vel_pct,dir_deg\n"
        char buffer[256];
        memset(buffer, 0, sizeof(buffer));
        ssize_t received = recv(sock_fd_, buffer, sizeof(buffer) - 1, 0);
        if (received <= 0) {
            std::cerr << "[EV3Driver] Error al recibir: " << strerror(errno) << std::endl;
            return false;
        }

        // ── 4. Parsea la respuesta y convierte a radianes ─────────────────────
        std::string response(buffer);
        if (!parseResponse(response)) {
            return false;
        }

        return true;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Getters — equivalentes a getPositionRadian() y getVelocityRadianPerSec()
    // Estos van dentro de read() del Hardware Component
    // ─────────────────────────────────────────────────────────────────────────

    double getTractionPositionRad()  const { return traction_position_rad_;  }
    double getTractionVelocityRadS() const { return traction_velocity_rad_s_; }
    double getSteeringPositionRad()  const { return steering_position_rad_;  }

    // ─────────────────────────────────────────────────────────────────────────
    // isConnected() — útil para verificar estado antes de operar
    // ─────────────────────────────────────────────────────────────────────────
    bool isConnected() const { return sock_fd_ >= 0; }

private:

    // ─────────────────────────────────────────────────────────────────────────
    // parseResponse() — convierte el string del EV3 a valores en radianes
    // formato entrada: "pos_counts,vel_pct,dir_deg\n"
    // ─────────────────────────────────────────────────────────────────────────
    bool parseResponse(const std::string & response)
    {
        try {
            std::stringstream ss(response);
            std::string token;

            // posición de tracción: counts → radianes
            std::getline(ss, token, ',');
            double pos_counts = std::stod(token);
            traction_position_rad_ = pos_counts * (2.0 * M_PI / COUNTS_PER_REV);

            // velocidad de tracción: porcentaje → rad/s
            std::getline(ss, token, ',');
            double vel_pct = std::stod(token);
            traction_velocity_rad_s_ = (traction_type_ == EV3MotorType::LARGE)
                                       ? vel_pct / 100.0 * LARGE_MOTOR_MAX_RAD_S
                                       : vel_pct / 100.0 * MEDIUM_MOTOR_MAX_RAD_S;

            // posición de dirección: grados → radianes
            std::getline(ss, token);
            double dir_deg = std::stod(token);
            steering_position_rad_ = dir_deg * (M_PI / 180.0);

        } catch (const std::exception & e) {
            std::cerr << "[EV3Driver] Error al parsear respuesta: " << e.what()
                      << " | respuesta: " << response << std::endl;
            return false;
        }

        return true;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Variables privadas
    // ─────────────────────────────────────────────────────────────────────────

    // conexión
    std::string    ip_;
    int            port_;
    int            sock_fd_;
    EV3MotorType   traction_type_;

    // comandos que se van a enviar (los escribe el Hardware Component)
    double traction_cmd_rad_s_ = 0.0;
    double steering_cmd_rad_   = 0.0;

    // estados leídos del EV3 (los lee el Hardware Component)
    double traction_position_rad_   = 0.0;
    double traction_velocity_rad_s_ = 0.0;
    double steering_position_rad_   = 0.0;
};

#endif  // EV3_DRIVER_HPP
