
make
if [ $? -ne 0 ]; then
    echo "Make failed. Exiting."
    exit 1
fi
echo "Running MCLsim"
./build/MCLsim