import { defineConfig } from 'vite';

export default defineConfig({
  base: '/PivotSim/',
  test: {
    environment: 'jsdom',
  },
});
