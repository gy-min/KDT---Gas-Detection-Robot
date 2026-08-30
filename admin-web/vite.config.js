import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      // CCTV 스트리밍 서버(server/main.py, 로컬 테스트 8081)로 넘깁니다.
      // 프록시를 쓰면 프론트와 같은 출처가 되어 CORS 설정이 필요 없고,
      // 코드에 서버 IP를 하드코딩하지 않아도 됩니다.
      '/api': {
        target: 'http://localhost:8081',
        changeOrigin: true,
      },
    },
  },
})
