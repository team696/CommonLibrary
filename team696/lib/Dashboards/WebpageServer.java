package frc.team696.lib.Dashboards;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.nio.file.Files;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;

import frc.team696.lib.Logging.PLog;
import frc.team696.robot.Robot;

public class WebpageServer {
    public static final int PORT = 5806;
    private static HttpServer _server;
    private static String baseDir;

    public static void start() {
        try {
            if (Robot.isSimulation()) {
                baseDir = "src/main/deploy/webpageFiles";
            } else {
                baseDir = "home/lvuser/deploy/webpageFiles";
            }

            _server = HttpServer.create(new InetSocketAddress(PORT), 0);

            // Set up a context to listen to root requests "/"
            _server.createContext("/", new RootHandler());
            
            // Start the server
            _server.setExecutor(null);  // Use the default executor
            _server.start();
            
            PLog.info("WebPage","Server is running on port "+PORT+".");
        } catch (Exception e) {

        }
    }

    static class RootHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {

            String requestMethod = exchange.getRequestMethod();
            if ("GET".equals(requestMethod)) {
                // Get the requested path
                String filePath = exchange.getRequestURI().getPath();

                // Default to index.html if no file is specified
                if (filePath.equals("/")) {
                    filePath = "/main.html";
                }

                File file = new File(baseDir + filePath);
                
                if (!file.exists()) PLog.info("hi", "no");

                FileInputStream fis = new FileInputStream(file);
                byte[] fileContent = new byte[(int) file.length()];
                fis.read(fileContent);
                fis.close();

                // Send HTTP headers (200 OK)
                String contentType = Files.probeContentType(file.toPath());
                exchange.getResponseHeaders().set("Content-Type", contentType);
                exchange.sendResponseHeaders(200, fileContent.length);

                // Write the HTML content to the response
                OutputStream os = exchange.getResponseBody();
                os.write(fileContent);
                os.close();
            }
        }
    }
}
