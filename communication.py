#########################################################################
#                                                                       #
#   SECURE DRONE SWARM SYSTEM - CORE MODULE                             #
#                                                                       #
#   Developer : Md Shahanur Islam Shagor                                #
#   Role      : Project Architect & Lead Developer                      #
#   Version   : 1.0.2                                                   #
#   Status    : Production Ready                                        #
#                                                                       #
#   "Protecting the skies with decentralized intelligence."             #
#                                                                       #
#########################################################################
"""
Secure Communication Module - AES encryption for drone-to-drone communication
No IP/Internet required - direct peer-to-peer
"""

import socket
import threading
import json
import time
import logging
from typing import Optional, Callable, Dict
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import hashes, hmac
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
import os
import struct

class MessageType:
    """Message types for drone communication"""
    HEARTBEAT = "heartbeat"
    STATUS_UPDATE = "status_update"
    COMMAND = "command"
    ELECTION_VOTE = "election_vote"
    ELECTION_RESULT = "election_result"
    POSITION_UPDATE = "position_update"
    EMERGENCY = "emergency"

class SecureCommunication:
    """
    Secure communication channel for drone swarm
    Uses AES-256 encryption with HMAC authentication
    """
    
    def __init__(self, drone_id: int, swarm_key: str = "default_swarm_key_change_this"):
        self.drone_id = drone_id
        
        # Generate encryption key from swarm password
        self.key = self._derive_key(swarm_key)
        
        # Communication state
        self.running = False
        self.message_handlers: Dict[str, Callable] = {}
        
        # Socket for receiving
        self.recv_socket = None
        self.recv_thread = None
        
        # Multicast settings (for local network without IP)
        self.multicast_group = "224.0.0.251"  # Local multicast
        self.multicast_port = 5000 + drone_id
        
        # Message sequence
        self.message_sequence = 0
        self.received_sequences = {}  # Track received messages to prevent duplicates
        
        # Logging
        self.logger = logging.getLogger(f"SecureComm_Drone{drone_id}")
        self.setup_logging()
    
    def setup_logging(self):
        """Configure logging"""
        handler = logging.FileHandler(f'logs/comm_drone_{self.drone_id}.log')
        handler.setFormatter(logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        ))
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)
    
    def _derive_key(self, password: str) -> bytes:
        """Derive encryption key from password using PBKDF2HMAC"""
        salt = b"drone_swarm_salt_2024"  # In production, use random salt
        kdf = PBKDF2HMAC(
            algorithm=hashes.SHA256(),
            length=32,
            salt=salt,
            iterations=100000,
            backend=default_backend()
        )
        return kdf.derive(password.encode())
    
    def start(self):
        """Start communication system"""
        if self.running:
            return
        
        self.running = True
        
        # Start receiver thread
        self.recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.recv_thread.start()
        
        self.logger.info(f"Communication started on port {self.multicast_port}")
    
    def stop(self):
        """Stop communication"""
        self.running = False
        
        if self.recv_socket:
            self.recv_socket.close()
        
        if self.recv_thread:
            self.recv_thread.join(timeout=2.0)
        
        self.logger.info("Communication stopped")
    
    def _receive_loop(self):
        """Receive and decrypt messages"""
        # Create UDP socket
        self.recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.recv_socket.bind(('', self.multicast_port))
            
            # Join multicast group
            mreq = struct.pack("4sl", socket.inet_aton(self.multicast_group), socket.INADDR_ANY)
            self.recv_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            
        except Exception as e:
            self.logger.error(f"Failed to bind socket: {e}")
            return
        
        while self.running:
            try:
                data, addr = self.recv_socket.recvfrom(4096)
                self.logger.info(
                    f"RX encrypted from {addr[0]}:{addr[1]} bytes={len(data)} payload_hex={data.hex()}"
                )
                
                # Decrypt and process
                message = self._decrypt_message(data)
                if message:
                    self._handle_received_message(message, addr)
                    
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.logger.error(f"Receive error: {e}")
    
    def _encrypt_message(self, message: dict) -> bytes:
        """
        Encrypt message with AES-256-GCM
        Format: [IV(16) | Encrypted Data | Tag(16)]
        """
        try:
            # Add metadata
            message["timestamp"] = time.time()
            message["sender_id"] = self.drone_id
            message["sequence"] = self.message_sequence
            self.message_sequence += 1
            
            # Convert to JSON
            plaintext = json.dumps(message).encode()
            
            # Generate random IV
            iv = os.urandom(16)
            
            # Encrypt with AES-GCM
            cipher = Cipher(
                algorithms.AES(self.key),
                modes.GCM(iv),
                backend=default_backend()
            )
            encryptor = cipher.encryptor()
            ciphertext = encryptor.update(plaintext) + encryptor.finalize()
            
            # Combine IV + ciphertext + tag
            return iv + ciphertext + encryptor.tag
            
        except Exception as e:
            self.logger.error(f"Encryption error: {e}")
            return None
    
    def _decrypt_message(self, encrypted_data: bytes) -> Optional[dict]:
        """Decrypt received message"""
        try:
            # Extract components
            iv = encrypted_data[:16]
            tag = encrypted_data[-16:]
            ciphertext = encrypted_data[16:-16]
            
            # Decrypt
            cipher = Cipher(
                algorithms.AES(self.key),
                modes.GCM(iv, tag),
                backend=default_backend()
            )
            decryptor = cipher.decryptor()
            plaintext = decryptor.update(ciphertext) + decryptor.finalize()
            
            # Parse JSON
            message = json.loads(plaintext.decode())
            self.logger.info(
                f"RX decrypted sender={message.get('sender_id')} seq={message.get('sequence')} "
                f"type={message.get('type')} data={json.dumps(message.get('data', {}), default=str)}"
            )
            
            # Check if we've seen this message before (duplicate prevention)
            sender_id = message.get("sender_id")
            sequence = message.get("sequence")
            
            if sender_id in self.received_sequences:
                if sequence <= self.received_sequences[sender_id]:
                    return None  # Duplicate message
            
            self.received_sequences[sender_id] = sequence
            
            return message
            
        except Exception as e:
            self.logger.warning(f"Decryption error: {e}")
            return None
    
    def _handle_received_message(self, message: dict, addr):
        """Process received message"""
        # Don't process own messages
        if message.get("sender_id") == self.drone_id:
            return
        
        msg_type = message.get("type")
        
        if msg_type in self.message_handlers:
            try:
                self.message_handlers[msg_type](message, addr)
            except Exception as e:
                self.logger.error(f"Handler error for {msg_type}: {e}")
    
    def register_handler(self, message_type: str, handler: Callable):
        """Register message handler"""
        self.message_handlers[message_type] = handler
        self.logger.info(f"Registered handler for {message_type}")
    
    def send_message(self, message_type: str, data: dict, target_port: Optional[int] = None):
        """
        Send encrypted message to swarm
        
        Args:
            message_type: Type of message
            data: Message payload
            target_port: Specific port or None for broadcast
        """
        message = {
            "type": message_type,
            "data": data
        }
        
        encrypted = self._encrypt_message(message)
        if not encrypted:
            return False
        
        target_desc = str(target_port) if target_port else "broadcast:5000-5009"
        self.logger.info(
            f"TX encrypted type={message_type} target={target_desc} bytes={len(encrypted)} "
            f"payload_hex={encrypted.hex()}"
        )
        
        # Create send socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        
        try:
            if target_port:
                # Send to specific drone
                sock.sendto(encrypted, (self.multicast_group, target_port))
            else:
                # Broadcast to all drones (ports 5000-5010)
                for port in range(5000, 5010):
                    if port != self.multicast_port:  # Don't send to self
                        try:
                            sock.sendto(encrypted, (self.multicast_group, port))
                        except:
                            pass
            
            return True
            
        except Exception as e:
            self.logger.error(f"Send error: {e}")
            return False
        finally:
            sock.close()
    
    def broadcast_heartbeat(self, drone_status: dict):
        """Broadcast heartbeat with status"""
        self.send_message(MessageType.HEARTBEAT, drone_status)
    
    def send_position_update(self, position: dict):
        """Send position update"""
        self.send_message(MessageType.POSITION_UPDATE, position)
    
    def send_emergency_signal(self, reason: str):
        """Send emergency signal"""
        self.send_message(MessageType.EMERGENCY, {"reason": reason})
    
    def send_election_vote(self, candidate_id: int, score: float):
        """Send election vote"""
        data = {
            "candidate_id": candidate_id,
            "score": score
        }
        self.send_message(MessageType.ELECTION_VOTE, data)
    
    def announce_leader(self, leader_id: int):
        """Announce new leader"""
        data = {
            "leader_id": leader_id
        }
        self.send_message(MessageType.ELECTION_RESULT, data)


class CommunicationManager:
    """
    Manages communication for multiple drones in simulation
    In real deployment, each drone would have its own SecureCommunication instance
    """
    
    def __init__(self):
        self.comm_modules: Dict[int, SecureCommunication] = {}
        self.logger = logging.getLogger("CommunicationManager")
    
    def add_drone_comm(self, drone_id: int, swarm_key: str = "default_swarm_key"):
        """Add communication module for a drone"""
        if drone_id not in self.comm_modules:
            comm = SecureCommunication(drone_id, swarm_key)
            self.comm_modules[drone_id] = comm
            comm.start()
            self.logger.info(f"Communication module added for drone {drone_id}")
    
    def remove_drone_comm(self, drone_id: int):
        """Remove drone communication"""
        if drone_id in self.comm_modules:
            self.comm_modules[drone_id].stop()
            del self.comm_modules[drone_id]
            self.logger.info(f"Communication module removed for drone {drone_id}")
    
    def get_comm(self, drone_id: int) -> Optional[SecureCommunication]:
        """Get communication module for drone"""
        return self.comm_modules.get(drone_id)
    
    def stop_all(self):
        """Stop all communication"""
        for comm in self.comm_modules.values():
            comm.stop()
        self.comm_modules.clear()
