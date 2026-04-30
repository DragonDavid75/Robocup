from http.server import HTTPServer, BaseHTTPRequestHandler
#import http.server
#import socketserver

PORT = 8080

class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
            print("got get\n")
        elif self.path == '/index.html':
          print("got index\n")
          try:
            file_html = open("index.html").read()
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.send_header('Content-Length', len(file_html))
            self.end_headers()
            self.wfile.write(bytes(file_html, 'utf-8'))
          except:
            self.send_response(404)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b'404 - Not Found')
        else:
          print("got to else\n")
          self.send_error(404)
          self.end_headers()

httpd = HTTPServer(('', PORT), SimpleHTTPRequestHandler)
httpd.serve_forever()
