import http.server
import ssl

# Define the specific IP address and port
server_address = ('172.20.10.13', 8000)

# Set up the HTTP server
httpd = http.server.HTTPServer(server_address, http.server.SimpleHTTPRequestHandler)

# Create an SSL context
ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ssl_context.load_cert_chain(certfile="server.crt", keyfile="server.key")

# Wrap the server socket with SSL
httpd.socket = ssl_context.wrap_socket(httpd.socket, server_side=True)

print("Serving on https://172.20.10.13:8000")
httpd.serve_forever()
