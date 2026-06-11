"""EHB (Electro-Hydraulic Brake) driver for brake control via CAN bus.

This module provides a hardware abstraction layer for controlling the vehicle's
electro-hydraulic brake system over CAN bus. It handles low-level CAN communication,
message building/parsing, timing, and error handling. Includes Hardware Pedal Override.
"""

from typing import Optional
import logging
import threading
import time


class EHBDriver:
    # CAN configuration
    DEFAULT_CAN_INTERFACE = 'socketcan'
    DEFAULT_CAN_CHANNEL = 'can2'
    
    # Message IDs
    MSG_ID_PRESSURE = 0x150
    MSG_ID_STATUS = 0x152
    MSG_ID_FEEDBACK = 0x182
    
    # Timing constants (in seconds)
    TX_PERIOD = 0.02  # 50Hz transmission rate for messages
    RX_TIMEOUT = 0.1  # 100ms timeout for receiving feedback
    ERROR_THRESHOLD = 100  # Error count before connection failure
    ERROR_RECOVERY_CREDIT = 3  # Error count reduction per successful message
    
    # Pressure scaling
    PRESSURE_SCALE = 40.0  # Multiplier for pressure values
    PRESSURE_RESOLUTION = 0.02  # Bar per bit
    
    def __init__(
        self,
        can_channel: str = DEFAULT_CAN_CHANNEL,
        can_interface: str = DEFAULT_CAN_INTERFACE,
        pressure_scale: float = PRESSURE_SCALE,
        mock_mode: bool = False,
        pedal_can_id: int = 0x180,
        pedal_byte_index: int = 1,
        pedal_threshold: int = 12,
    ):
        self.can_channel = can_channel
        self.can_interface = can_interface
        self.pressure_scale = pressure_scale
        self.mock_mode = mock_mode
        
        # Hardware Override Parameters
        self.pedal_can_id = pedal_can_id
        self.pedal_byte_index = pedal_byte_index
        self.pedal_threshold = pedal_threshold
        
        self._bus: Optional[object] = None
        self._connected = False
        self._last_pressure = 0.0
        
        # Override States
        self._physical_pedal_raw = 0
        self._pedal_override_active = False
        
        # Message counters and switch bits
        self._msg_counter = 0
        self._switch_bit1 = 0
        self._switch_bit2 = 1
        
        # CAN message tracking
        self._last_can_message_time: Optional[float] = None
        self._can_timeout_sec = 0.5
        
        self._tx_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        
        self.logger = logging.getLogger('EHBDriver')
        
        self._initialize_can()
        if self._connected or self.mock_mode:
            self._start_threads()
    
    def _initialize_can(self) -> bool:
        if self.mock_mode:
            self.logger.info("Running in MOCK mode - no actual CAN communication")
            self._connected = True
            return True
        
        try:
            import can
            self._bus = can.interface.Bus(
                channel=self.can_channel,
                bustype=self.can_interface,
                receive_own_messages=True
            )
            self._connected = True
            self.logger.info(f"CAN bus initialized: {self.can_channel} via {self.can_interface}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize CAN bus: {e}")
            self._connected = False
            return False
    
    def _start_threads(self) -> None:
        if self._running: return
        self._running = True
        
        self._tx_thread = threading.Thread(target=self._tx_loop, name='ehb-tx', daemon=True)
        self._tx_thread.start()
        
        self._rx_thread = threading.Thread(target=self._rx_loop, name='ehb-rx', daemon=True)
        self._rx_thread.start()
    
    def _tx_loop(self) -> None:
        while self._running:
            try:
                self._send_messages()
                time.sleep(self.TX_PERIOD)
            except Exception as e:
                self.logger.error(f"Error in TX loop: {e}")
                time.sleep(self.TX_PERIOD)
    
    def _rx_loop(self) -> None:
        if self.mock_mode:
            while self._running:
                time.sleep(self.RX_TIMEOUT)
                with self._lock:
                    self._last_can_message_time = time.time()
            return
        
        while self._running:
            try:
                if not self._connected:
                    time.sleep(self.RX_TIMEOUT)
                    continue
                
                message = self._bus.recv(timeout=self.RX_TIMEOUT)
                
                if message is not None:
                    with self._lock:
                        self._last_can_message_time = time.time()
                        
                        # HARDWARE OVERRIDE DETECTION: Detecteer of mens fysiek remt
                        if message.arbitration_id == self.pedal_can_id:
                            if len(message.data) > self.pedal_byte_index:
                                self._physical_pedal_raw = message.data[self.pedal_byte_index]
                                self._pedal_override_active = (self._physical_pedal_raw > self.pedal_threshold)
                        
            except Exception as e:
                self.logger.debug(f"RX error: {e}")
    
    def _send_messages(self) -> None:
        with self._lock:
            # === DE ARBITER ===
            # Als de mens het pedaal intrapt, negeer self._last_pressure en gebruik de fysieke waarde!
            if self._pedal_override_active:
                # Schaal 0-255 naar 0.0-1.0
                pressure = max(0.0, min(1.0, self._physical_pedal_raw / 255.0))
            else:
                pressure = self._last_pressure
            
            self._msg_counter = (self._msg_counter + 1) % 16
            if self._switch_bit1 == 0:
                self._switch_bit1 = 1
                self._switch_bit2 = 0
            else:
                self._switch_bit1 = 0
                self._switch_bit2 = 1
            
            msg_data_150 = self._build_message_150(pressure, self._switch_bit1, self._switch_bit2)
            msg_data_152 = self._build_message_152(self._msg_counter)
        
        if self.mock_mode or not self._connected: return
        
        try:
            import can
            msg_150 = can.Message(arbitration_id=self.MSG_ID_PRESSURE, data=msg_data_150, is_extended_id=False, is_rx=False)
            msg_152 = can.Message(arbitration_id=self.MSG_ID_STATUS, data=msg_data_152, is_extended_id=False, is_rx=False)
            self._bus.send(msg_152)
            self._bus.send(msg_150)
        except Exception as e:
            self.logger.error(f"Failed to send CAN messages: {e}")
    
    def _build_message_150(self, pressure: float, bit1: int, bit2: int) -> bytearray:
        pressure_front = int(self.pressure_scale * pressure)
        pressure_val = int(pressure_front / self.PRESSURE_RESOLUTION)
        msg_data = bytearray(8)
        msg_data[7] = pressure_val & 0xFF
        msg_data[6] = ((pressure_val & 0x3F00) >> 8) | ((bit1 & 0x01) << 6) | (1 << 7)
        msg_data[5] = pressure_val & 0xFF
        msg_data[4] = ((pressure_val & 0x3F00) >> 8) | ((bit2 & 0x01) << 6) | (1 << 7)
        msg_data[3] = pressure_val & 0xFF
        msg_data[2] = ((pressure_val & 0x3F00) >> 8) | (1 << 6) | (1 << 7)
        msg_data[1] = pressure_val & 0xFF
        msg_data[0] = ((pressure_val & 0x3F00) >> 8) | (1 << 6) | (1 << 7)
        return msg_data
    
    def _build_message_152(self, counter: int) -> bytearray:
        msg_data = bytearray(8)
        msg_data[0] = (counter & 0x0F) | (1 << 4) | (0 << 5) | (0 << 6) | (0 << 7)
        msg_data[1] = 0 | (1 << 1) | (0 << 2) | (1 << 4) | (0 << 5) | (0 << 6)
        msg_data[2] = (0 << 6) | ((int(0 / 0.0625) & 0x1F00) >> 8)
        msg_data[3] = int(0 / 0.0625) & 0xFF
        msg_data[4] = 0 | (0 << 4) | (0 << 5) | (0 << 6)
        msg_data[5] = int(1000 / 32) & 0xFF
        return msg_data
    
    def set_pressure(self, pressure: float) -> bool:
        if not (0.0 <= pressure <= 1.0): return False
        with self._lock:
            self._last_pressure = pressure
        return True
    
    # === NIEUWE FUNCTIES VOOR DE ROS NODE ===
    def is_override_active(self) -> bool:
        with self._lock:
            return self._pedal_override_active
            
    def get_physical_pressure_raw(self) -> int:
        with self._lock:
            return self._physical_pedal_raw

    def is_connected(self) -> bool:
        with self._lock:
            if not self._connected: return False
            if self._last_can_message_time is None: return True
            return (time.time() - self._last_can_message_time) < self._can_timeout_sec
    
    def get_time_since_last_message(self) -> Optional[float]:
        with self._lock:
            if self._last_can_message_time is None: return None
            return time.time() - self._last_can_message_time
    
    def has_can_communication(self) -> bool:
        return self.is_connected()
    
    def stop(self) -> bool:
        return self.set_pressure(0.0)
    
    def close(self) -> None:
        self.set_pressure(0.0)
        self._running = False
        if self._tx_thread: self._tx_thread.join(timeout=1.0)
        if self._rx_thread: self._rx_thread.join(timeout=1.0)
        if self._bus is not None and not self.mock_mode:
            try: self._bus.shutdown()
            except: pass
        self._connected = False
