BOARDS = {
    'arduino': {
        'digital': tuple(x for x in range(14)),
        'analog': tuple(x for x in range(6)),
        'pwm': (3, 5, 6, 9, 10, 11),
        'serial_rx': (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12),
        'use_ports': True,
        'disabled': (0, 1)  # Rx, Tx, Crystal
    },
    'arduino_mega': {
        'digital': tuple(x for x in range(54)),
        'analog': tuple(x for x in range(16)),
        'pwm': tuple(x for x in range(2, 14)),
        'serial_rx': (10, 11, 12, 13, 14, 15, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69),
        'use_ports': True,
        'disabled': (0, 1)  # Rx, Tx, Crystal
    },
    'arduino_due': {
        'digital': tuple(x for x in range(54)),
        'analog': tuple(x for x in range(12)),
        'pwm': tuple(x for x in range(2, 14)),
        'serial_rx': (10, 11, 12, 13, 14, 15, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69), #not confirmed
        'use_ports': True,
        'disabled': (0, 1)  # Rx, Tx, Crystal
    },
    'arduino_nano': {
        'digital': tuple(x for x in range(14)),
        'analog': tuple(x for x in range(8)),
        'pwm': (3, 5, 6, 9, 10, 11),
        'serial_rx': (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12), # not confirmed
        'use_ports': True,
        'disabled': (0, 1)  # Rx, Tx, Crystal
    },
    'arduino_leonardo': {
        'digital': tuple(x for x in range(19)),
        'analog': tuple(x for x in range(11)),
        'pwm': (3, 5, 6, 9, 10, 11, 13),
        'serial_rx': (8, 9, 10, 11, 14, 15, 16),
        'use_ports': True,
        'disabled': (0, 1)  # Rx, Tx, Crystal        
    }
}
