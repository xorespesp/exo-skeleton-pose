import { defineConfig } from 'vite';
import fs from 'node:fs';
import path from 'node:path';

// The flatc-generated TypeScript uses ESM `.js` import specifiers that actually
// point at sibling `.ts` files (standard TS style). Vite/rollup don't rewrite
// `.js` -> `.ts` on their own, so this small resolver does it for relative imports.
const resolveTsFromJs = {
    name: 'resolve-ts-from-js',
    enforce: 'pre',
    resolveId(source, importer) {
        if (importer && source.startsWith('.') && source.endsWith('.js')) {
            const tsPath = path.resolve(path.dirname(importer), source).replace(/\.js$/, '.ts');
            if (fs.existsSync(tsPath)) { return tsPath; }
        }
        return null;
    },
};

// esnext keeps top-level await (used by three's WebGPU capability probe and our
// async model load) intact instead of erroring at build time.
export default defineConfig({
    plugins: [resolveTsFromJs],
    esbuild: { target: 'esnext' },
    build: { target: 'esnext' },
});
