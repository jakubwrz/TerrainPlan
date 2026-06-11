from bu03_util import BU03Device, PORT

def print_header(title):
    print("\n" + "="*50)
    print(f" {title}")
    print("="*50)

def main():
    print_header("UWB Long-Range Configuration Tool")
    print("This tool will configure your BU03 modules for maximum range:")
    print(" - Frequency: Channel 5 (6.5 GHz) for better obstacle penetration")
    print(" - Data Rate: 850 kbps for massive sensitivity/range boost\n")
    
    print("What are you programming right now?")
    print("  [0] The Rover (Tag)")
    print("  [1] Anchor/Mast 0 (Origin)")
    print("  [2] Anchor/Mast 1 (Y-Axis)")
    print("  [3] Anchor/Mast 2 (X-Axis)")
    
    choice = input("\nEnter choice (0-3): ").strip()
    
    if choice == '0':
        device_id = 0
        role = 0  # 0 = Tag
        name = "Rover (Tag)"
    elif choice == '1':
        device_id = 0
        role = 1  # 1 = Base Station
        name = "Anchor 0"
    elif choice == '2':
        device_id = 1
        role = 1
        name = "Anchor 1"
    elif choice == '3':
        device_id = 2
        role = 1
        name = "Anchor 2"
    else:
        print("Invalid choice. Exiting.")
        return

    channel = 1  # 1 = Channel 5
    rate = 0     # 0 = 850 kbps

    print(f"\nConnecting to BU03 on {PORT}...")
    try:
        device = BU03Device(port=PORT)
    except Exception as e:
        print(f"\n❌ Error connecting to COM port: {e}")
        print("Make sure the module is plugged in via USB and no other program (like uwb_server.py) is using the port!")
        return

    print(f"✓ Connected!")
    print(f"\nApplying Long-Range Settings for: {name}")
    print(f"-> ID: {device_id}, Role: {role}, Channel: 5, Rate: 850k")
    
    # Apply configuration
    response = device.set_config(device_id=device_id, role=role, channel=channel, rate=rate, save=True)
    
    if "OK" in response or "setcfg" in response.lower() or "id:" in response.lower():
        print("\n✅ SUCCESS! The device is rebooting to apply the new settings.")
        print("You can now unplug this module and plug in the next one.")
    else:
        print(f"\n⚠️ Unexpected response: {response}")
        print("Please try again.")

if __name__ == "__main__":
    main()
