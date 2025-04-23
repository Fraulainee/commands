# Host React App + Vite in Hostinger

### Build your Application
```
npm run build
```

### Hostinger Vite path problem
If the file doesn’t exist, serve index.html instead
```
<IfModule mod_rewrite.c>
  RewriteEngine On
  RewriteBase /
  RewriteRule ^index\.html$ - [L]
  RewriteCond %{REQUEST_FILENAME} !-f
  RewriteCond %{REQUEST_FILENAME} !-d
  RewriteRule . /index.html [L]
</IfModule>
```
