"""
Script client pour PC - Réception du streaming caméra RealSense
À exécuter sur votre PC Windows
"""
import socket
import pickle
import struct
import cv2
import numpy as np
import argparse

# Configuration par défaut
DEVICES = {
    'jetson': {'ip': '192.168.55.1', 'name': 'Jetson Nano'},
    'raspberry': {'ip': '10.47.126.153', 'name': 'Raspberry Pi'},
}
DEFAULT_DEVICE = 'raspberry'
PORT = 8485

def receive_stream(device_ip, device_name):
    """Se connecte au serveur et affiche le stream"""
    # Création du socket client
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    print(f"Connexion à {device_name} ({device_ip}:{PORT})...")
    
    try:
        client_socket.connect((device_ip, PORT))
        print(f"✓ Connecté à {device_name}")
        print("Appuyez sur 'q' pour quitter")
        
        data = b""
        payload_size = struct.calcsize("!I")  # 4-byte network-order header
        
        while True:
            # Récupère la taille du message
            while len(data) < payload_size:
                packet = client_socket.recv(4096)
                if not packet:
                    break
                data += packet
            
            if not data:
                break
            
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("!I", packed_msg_size)[0]
            
            # Récupère les données de l'image
            while len(data) < msg_size:
                data += client_socket.recv(4096)
            
            frame_data = data[:msg_size]
            data = data[msg_size:]
            
            # Décode l'image
            frame = pickle.loads(frame_data)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            
            # Affiche l'image
            cv2.imshow(f'RealSense Stream - {device_name}', frame)
            
            # Quitte si 'q' est pressé
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except ConnectionRefusedError:
        print(f"✗ Impossible de se connecter à {device_name} ({device_ip}:{PORT})")
        print("Vérifiez que :")
        print(f"  1. Le serveur tourne sur {device_name}")
        print("  2. L'IP est correcte")
        print("  3. Le port n'est pas bloqué par un firewall")
    
    except KeyboardInterrupt:
        print("\n✓ Arrêt du client...")
    
    finally:
        client_socket.close()
        cv2.destroyAllWindows()
        print("✓ Client fermé")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Client RealSense - Streaming depuis Jetson/Raspberry')
    parser.add_argument('--device', '-d', 
                        choices=['jetson', 'raspberry'], 
                        default=DEFAULT_DEVICE,
                        help='Appareil cible (jetson ou raspberry)')
    parser.add_argument('--ip', 
                        help='IP personnalisée (remplace la config par défaut)')
    
    args = parser.parse_args()
    
    # Récupérer la config
    if args.ip:
        device_ip = args.ip
        device_name = f"Device ({args.ip})"
    else:
        config = DEVICES[args.device]
        device_ip = config['ip']
        device_name = config['name']
    
    print("=" * 50)
    print(f"  CLIENT REALSENSE - {device_name.upper()}")
    print("=" * 50)
    print(f"Appareil: {device_name}")
    print(f"IP: {device_ip}:{PORT}")
    print("=" * 50)
    
    receive_stream(device_ip, device_name)
