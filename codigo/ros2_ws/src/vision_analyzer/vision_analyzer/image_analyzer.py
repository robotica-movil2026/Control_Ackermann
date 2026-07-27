import os
from io import BytesIO

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

import cv_bridge
from PIL import Image as PILImage
from google import genai


class ImageAnalyzer(Node):

    def __init__(self):
        super().__init__('image_analyzer')
        self._api_key = self.declare_parameter('api_key', '').value
        if not self._api_key:
            self._api_key = os.environ.get('GEMINI_API_KEY', '')
        self._model = self.declare_parameter('model', 'gemini-3.5-flash').value
        self._max_dim = self.declare_parameter('max_dimension', 640).value
        self._prompt = self.declare_parameter(
            'prompt',
            'Ignora completamente el soporte fisico de la imagen (pantalla de celular, monitor, '
            'papel impreso, carton, etc.). Analiza SOLO la escena real mostrada y responde en '
            'JSON sin markdown:\n'
            '{\n'
            '  "tipo_lugar": "Tienda | Ferreteria | Panaderia | Drogueria | Restaurante | Casa | Centro comercial | '
            'Edificio industrial | Oficina | Parque | Otro",\n'
            '  "descripcion": "texto breve (~15 palabras) con los elementos mas representativos ignorando los elementos moviles (bicicletas, carros, personas)",\n'
            '}'
        ).value

        self._bridge = cv_bridge.CvBridge()
        self._latest = None
        self._sub = self.create_subscription(Image, '/image_raw', self._cb, 10)
        self._pub = self.create_publisher(String, '/scene_analysis', 10)
        self._srv = self.create_service(Trigger, '/analyze_scene', self._analyze_cb)

        self._client = genai.Client(api_key=self._api_key) if self._api_key else None

        if not self._api_key:
            self.get_logger().error('No API key. Set GEMINI_API_KEY env var.')
        else:
            self.get_logger().info('Ready — call /analyze_scene')

    def _cb(self, msg):
        self._latest = msg

    def _analyze_cb(self, req, res):
        if self._latest is None:
            res.success = False
            res.message = 'No image received yet'
            return res
        if self._client is None:
            res.success = False
            res.message = 'No API key configured'
            return res

        try:
            cv_img = self._bridge.imgmsg_to_cv2(self._latest, 'bgr8')
            h, w = cv_img.shape[:2]
            if max(h, w) > self._max_dim:
                scale = self._max_dim / max(h, w)
                cv_img = cv2.resize(cv_img, (int(w * scale), int(h * scale)),
                                     interpolation=cv2.INTER_AREA)
            _, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            pil_img = PILImage.open(BytesIO(buf.tobytes()))

            response = self._client.models.generate_content(
                model=self._model,
                contents=[self._prompt, pil_img],
            )

            text = response.text
            self._pub.publish(String(data=text))
            self.get_logger().info(f'Gemini: {text}')

            res.success = True
            res.message = text
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            res.success = False
            res.message = str(e)
        return res


def main():
    rclpy.init()
    node = ImageAnalyzer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
