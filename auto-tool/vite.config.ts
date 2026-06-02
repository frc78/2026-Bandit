import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));

function saveAutoPlugin() {
  return {
    name: 'save-auto',
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    configureServer(server: any) {
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      server.middlewares.use('/api/save', (req: any, res: any) => {
        if (req.method !== 'POST') {
          res.statusCode = 405;
          res.end('Method Not Allowed');
          return;
        }
        let body = '';
        // eslint-disable-next-line @typescript-eslint/no-explicit-any
        req.on('data', (chunk: any) => (body += chunk));
        req.on('end', () => {
          try {
            const { name, code, autoJson } = JSON.parse(body) as { name: string; code: string; autoJson: string };
            const dir = path.resolve(
              __dirname,
              '../src/main/java/frc/robot/auto/gui_tests'
            );
            fs.mkdirSync(dir, { recursive: true });
            const filePath = path.join(dir, `${name}.kt`);
            fs.writeFileSync(filePath, code, 'utf-8');
            const autosDir = path.resolve(__dirname, 'autos');
            fs.mkdirSync(autosDir, { recursive: true });
            fs.writeFileSync(path.join(autosDir, `${name}.json`), autoJson, 'utf-8');
            res.statusCode = 200;
            res.setHeader('Content-Type', 'application/json');
            res.end(JSON.stringify({ path: filePath }));
          } catch (e) {
            res.statusCode = 500;
            res.end(String(e));
          }
        });
      });

      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      server.middlewares.use('/api/list-autos', (req: any, res: any) => {
        if (req.method !== 'GET') { res.statusCode = 405; res.end(); return; }
        const autosDir = path.resolve(__dirname, 'autos');
        const names = fs.existsSync(autosDir)
          ? fs.readdirSync(autosDir).filter((f: string) => f.endsWith('.json')).map((f: string) => f.slice(0, -5)).sort()
          : [];
        res.setHeader('Content-Type', 'application/json');
        res.end(JSON.stringify(names));
      });

      // req.url here is the remainder after the mount path, e.g. "/MyAuto"
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      server.middlewares.use('/api/load-auto', (req: any, res: any) => {
        if (req.method !== 'GET') { res.statusCode = 405; res.end(); return; }
        const name = req.url?.slice(1);
        if (!name) { res.statusCode = 400; res.end('Missing name'); return; }
        const filePath = path.join(path.resolve(__dirname, 'autos'), `${name}.json`);
        if (!fs.existsSync(filePath)) { res.statusCode = 404; res.end('Not found'); return; }
        res.setHeader('Content-Type', 'application/json');
        res.end(fs.readFileSync(filePath, 'utf-8'));
      });
    },
  };
}

export default defineConfig({
  plugins: [react(), saveAutoPlugin()],
});
