# Setup hurricane-template (Windows)

Follow these steps to set up your computer for building and uploading this code to the VEX V5 robot.

## 1. Get the Code

1. Open Terminal.
2. Install Git by running this command:
   ```
   winget install --id Git.Git -e --source winget
   ```
   If asked, approve any prompts to finish the installation.
3. Choose a folder where you want the code (for example, Desktop).
4. Run this command in Terminal to download the code:
   ```
   git clone https://github.com/Cunuduh/hurricane-template.git
   ```
5. Move into the newly created folder:
   ```
   cd hurricane-template
   ```

If you already have the code, make sure it's up to date by running this command inside the `hurricane-template` folder:
```
git pull
```

## 2. Install Rust

Go to [https://www.rust-lang.org/tools/install](https://www.rust-lang.org/tools/install) and follow the instructions to install Rust.

## 3. Set Up Rust for VEX

Copy and paste each line below into Terminal and press Enter after each:

```
rustup default nightly
rustup component add rust-src
cargo install cargo-v5
```

## 4. Upload Code to the Robot

1. Plug the robot brain or controller into your computer with a USB cable.
2. Make sure you are in the folder with your code (see step 1).
3. Build and upload the code by running:
   ```
   cargo v5 upload --release
   ```
4. Wait for the upload to finish. The robot will be ready to run the new code in slot 1, named "Hurricanes". If the upload fails while plugged into the controller, try unplugging and replugging the USB cable. If that doesn't work, plug the robot brain directly into your computer and try again.

If you see any errors unrelated to uploading the code, let me know!
