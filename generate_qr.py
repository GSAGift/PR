import qrcode

# Generate QR code for car 1
qr1 = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_L,
    box_size=10,
    border=4,
)
qr1.add_data('1')
qr1.make(fit=True)
img1 = qr1.make_image(fill_color="black", back_color="white")
img1.save("qr_1.png")

# Generate QR code for car 2
qr2 = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_L,
    box_size=10,
    border=4,
)
qr2.add_data('2')
qr2.make(fit=True)
img2 = qr2.make_image(fill_color="black", back_color="white")
img2.save("qr_2.png")

print("Generated qr_1.png and qr_2.png")