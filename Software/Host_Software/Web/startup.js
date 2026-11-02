// Keep startup instructions available when file:// blocks the module app.
(() => {
  if (location.protocol !== 'file:') return;

  const setText = (id, value) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  };
  const loading = document.getElementById('model-loading');
  if (loading) {
    const title = document.createElement('strong');
    title.textContent = '请通过启动器打开控制台';
    const instruction = document.createElement('small');
    instruction.textContent = '双击 Host_Software 文件夹中的 Launch_GUI.cmd';
    const detail = document.createElement('small');
    detail.textContent = '启动器会自动打开界面，并提供模型资源和设备连接服务';
    loading.replaceChildren(title, instruction, detail);
  }
  setText('connection-label', '等待启动本地服务');
  setText('model-status', '请运行 Launch_GUI.cmd');
  setText('footer-message', '当前为直接打开的 HTML 页面，请使用 Launch_GUI.cmd 启动上位机');
  document.body.dataset.launchRequired = 'true';
})();
